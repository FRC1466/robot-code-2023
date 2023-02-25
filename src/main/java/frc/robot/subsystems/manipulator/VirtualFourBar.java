package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import webblib.math.ArmPIDController;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder absoluteArmEncoder;
  private ArmPIDController armPID;
  private VirtualFourBarSimulation sim;

  public enum ARM {
    GROUND,
    VERTICAL,
    STORAGE,
    STATION,
    MID
  }

  private Rotation2d currentArm = Rotation2d.fromRadians(ArmConstants.minRadians);

  /** Create a new VirtualFourBar subsystem. */
  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    absoluteArmEncoder = new DutyCycleEncoder(ArmConstants.dutyCyclePort);
    absoluteArmEncoder.setDutyCycleRange(0, 1);
    absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new ArmPIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPID.setAvoidanceRange(
        Rotation2d.fromRadians(ArmConstants.maxRadians),
        Rotation2d.fromRadians(ArmConstants.minRadians));
    armPID.setTolerance(0.15);

    if (Robot.isSimulation()) {
      sim = new VirtualFourBarSimulation(absoluteArmEncoder);
      SmartDashboard.putData("Arm Sim", sim.getMech2d());
    }
  }

  @Override
  public void simulationPeriodic() {
    sim.update(armMotor.get());
  }

  /** Configure arm motor. */
  private void configArmMotor() {
    armMotor.configFactoryDefault();
    armMotor.configAllSettings(ArmConstants.ArmConfig.motorConfig);
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
            armPID.calculate(getPosition(), setpoint),
            -ArmConstants.armPosition.peakOutput,
            ArmConstants.armPosition.peakOutput);
    var feedforward = getPosition().getCos() * ArmConstants.gravityFF;
    SmartDashboard.putNumber("initial setpoint", setpoint.getRadians());
    SmartDashboard.putNumber("armPID error", armPID.getPositionError());
    SmartDashboard.putNumber("armPID output", motorOutput);
    SmartDashboard.putNumber("arm feedforward", feedforward);
    armMotor.set(motorOutput + feedforward);
  }

  public void setArm(ARM height) {
    switch (height) {
      case GROUND:
        currentArm = Rotation2d.fromRadians(ArmConstants.maxRadians);
        break;
      case STATION:
        currentArm = Rotation2d.fromDegrees(165);
        break;
      case MID:
        currentArm = Rotation2d.fromDegrees(165);
        break;
      case STORAGE:
        currentArm = Rotation2d.fromRadians(ArmConstants.minRadians);
        break;
      case VERTICAL:
        currentArm = Rotation2d.fromDegrees(90);
        break;
      default:
        throw new IllegalArgumentException("Height enum not supported.");
    }
  }

  public void ambientArm() {
    setArm(currentArm);
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
    // System.out.println("Output: " + armPID.calculate(Rotation2d.fromDegrees(180),
    // Rotation2d.fromDegrees(0)));
    SmartDashboard.putNumber("Proccessed Absolute Arm", getPosition().getRadians());
    SmartDashboard.putNumber("armPID error", armPID.getPositionError());
  }
}
