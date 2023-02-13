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

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder armEncoder;
  private PIDController armPID;

  private final DCMotor m_armGearbox;
  private final SingleJointedArmSim armSim;
  private final DutyCycleEncoderSim m_encoderSim;
    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_armPivot;
    private final MechanismLigament2d m_armTower;
    private final MechanismLigament2d m_arm;

  /** Create a new VirtualFourBar subsystem. */
  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    armEncoder = new DutyCycleEncoder(0);
    armEncoder.setDistancePerRotation(1);
    armEncoder.setPositionOffset(0.92);

    armPID =
        new PIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPID.setTolerance(0.01);

    m_armGearbox = DCMotor.getFalcon500(1);
    armSim =
    new SingleJointedArmSim(
        m_armGearbox, 16, 0.731599134, 0.6858, -Math.PI/2, 3 * Math.PI / 2, true, VecBuilder.fill(1 / 256));
    m_encoderSim = new DutyCycleEncoderSim(armEncoder);
      // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
      m_mech2d = new Mechanism2d(60, 60);
      m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
      m_armTower =
          m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
      m_arm =
          m_armPivot.append(
              new MechanismLigament2d(
                  "Arm",
                  30,
                  Units.radiansToDegrees(armSim.getAngleRads()),
                  6,
                  new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("Arm Sim", m_mech2d);
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
   * Set arm with PID.
   *
   * @param a setpoint in encoder units.
   */
  public void setArm(double a) {
    SmartDashboard.putNumber("setpoint", a);
    var motorOutput = MathUtil.clamp(armPID.calculate(armEncoder.getDistance(), a), -1, 1);
    SmartDashboard.putNumber("arm pid error", armPID.getPositionError());
    SmartDashboard.putNumber("armPID output", motorOutput);
    // armMotor.set(motorOutput + Rotation2d.fromRadians(armEncoder.getDistance()).getCos()*0.2);
  }

  public void setArmPercent(double a) {
    armMotor.set(a);
  }

  /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  public Boolean isAtSetpoint() {
    return armPID.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(armEncoder);
    SmartDashboard.putNumber("abs arm", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("norm arm", armEncoder.getDistance());
  }
}
