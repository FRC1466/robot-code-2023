package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import webblib.math.ArmPIDController;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder absoluteArmEncoder;
  private PIDController armPID;
  private ArmPIDController armPIDc;

  private DCMotor m_armGearbox;
  private SingleJointedArmSim armSim;
  private DutyCycleEncoderSim m_encoderSim;
  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_armPivot;
  private MechanismLigament2d m_armTower;
  private MechanismLigament2d m_arm;

  public enum HEIGHT {
    GROUND,
    TOP,
    STORAGE
  }

  /** Create a new VirtualFourBar subsystem. */
  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    absoluteArmEncoder = new DutyCycleEncoder(ArmConstants.dutyCyclePort);
    absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new PIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPID.setTolerance(0.01);
    armPIDc =
        new ArmPIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPIDc.setAvoidanceRange(
        Rotation2d.fromRadians(ArmConstants.maxRadians),
        Rotation2d.fromRadians(ArmConstants.minRadians));

    if (!Robot.isReal()) {
      createSimulation();
      SmartDashboard.putData("Arm Sim", m_mech2d);
    }
  }

  public void createSimulation() {
    m_armGearbox = DCMotor.getFalcon500(1);
    armSim =
        new SingleJointedArmSim(
            m_armGearbox,
            16,
            0.731599134,
            0.6858,
            -Math.PI / 2,
            3 * Math.PI / 2,
            true,
            VecBuilder.fill(1 / 256));
    m_encoderSim = new DutyCycleEncoderSim(absoluteArmEncoder);
    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    m_mech2d = new Mechanism2d(60, 60);
    m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
                "Arm",
                30,
                Units.radiansToDegrees(armSim.getAngleRads()),
                6,
                new Color8Bit(Color.kYellow)));
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armMotor.get() * 12);

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(armSim.getAngleRads());

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  /** Configure arm motor. */
  private void configArmMotor() {
    armMotor.configFactoryDefault();
    armMotor.configAllSettings(Robot.armConfig.config);
  }

  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition =
        absoluteArmEncoder.getAbsolutePosition() / ArmConstants.dutyCycleResolution;
    var adjustedPosition =
        Rotation2d.fromRotations(initialPosition)
            .minus(Rotation2d.fromRotations(ArmConstants.absolutePositionOffset));
    return adjustedPosition;
  }

  /**
   * Gets position of arm in radians. Assumes the arm being level within frame is the 0 point on the
   * x axis. Assumes CCW+.
   *
   * @return position in rad.
   */
  private Rotation2d getPosition() {
    return ArmConstants.encoderInverted
        ? getShiftedAbsoluteDistance().unaryMinus()
        : getShiftedAbsoluteDistance();
  }

  /**
   * Set arm with PID.
   *
   * @param setpoint setpoint in radians.
   */
  public void setArm(Rotation2d setpoint) {
    var motorOutput =
        MathUtil.clamp(
            armPIDc.calculate(getPosition(), setpoint),
            -ArmConstants.armPosition.peakOutput,
            ArmConstants.armPosition.peakOutput);
    var feedforward = getPosition().getCos() * ArmConstants.gravityFF;
    SmartDashboard.putNumber("initial setpoint", setpoint.getRadians());
    SmartDashboard.putNumber("armPID error", armPID.getPositionError());
    SmartDashboard.putNumber("armPID output", motorOutput);
    // armMotor.set(sanitizeMotorOutput(motorOutput + feedforward));
  }

  public void setArm(HEIGHT height) {
    switch (height) {
      case GROUND:
        setArm(Rotation2d.fromRadians(ArmConstants.minRadians));
        break;
      case TOP:
        setArm(Rotation2d.fromDegrees(90));
        break;
      case STORAGE:
        setArm(Rotation2d.fromRadians(ArmConstants.minRadians));
        break;
      default:
        throw new IllegalArgumentException("Height enum not supported.");
    }
  }

  /**
   * Sanitize motor input as pseudo-limit switches. If within the range defined in {@link
   * ArmConstants}, then ceases motor input unless its in the direction that's going away from the
   * range. Assumes motor output and encoders are CCW+, where up is forward from rest position in
   * robot.
   *
   * @param motorOutput current motor output to be sanitized.
   * @param currentPosition current position in radians.
   * @return sanitized output.
   */
  public double sanitizeMotorOutput(double motorOutput, double currentPosition) {
    if (currentPosition < ArmConstants.minRadians + ArmConstants.toleranceRadians) {
      return motorOutput > 0 ? motorOutput : 0;
    }
    if (currentPosition > ArmConstants.maxRadians - ArmConstants.toleranceRadians) {
      return motorOutput < 0 ? motorOutput : 0;
    }
    return motorOutput;
  }

  /**
   * Set arm motor percentage to a percent.
   *
   * @param percentOutput
   */
  public void setArmPercent(double percentOutput) {
    armMotor.set(percentOutput);
  }

  /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  public boolean isAtSetpoint() {
    return armPID.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(absoluteArmEncoder);
    SmartDashboard.putNumber("Raw Absolute Arm", absoluteArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Proccessed Absolute Arm", getPosition().getRadians());
  }
}
