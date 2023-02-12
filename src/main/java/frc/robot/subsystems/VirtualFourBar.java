package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder armEncoder;
  private PIDController armPID;

  /** Create a new VirtualFourBar subsystem. */
  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    armEncoder = new DutyCycleEncoder(0);
    armEncoder.setDistancePerRotation(1);
    armEncoder.setPositionOffset(0.52);

    armPID =
        new PIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPID.setTolerance(0.01);
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
    var motorOutput = MathUtil.clamp(armPID.calculate(armEncoder.getDistance(), a), -12, 12);
    SmartDashboard.putNumber("arm pid error", armPID.getPositionError());
    SmartDashboard.putNumber("armPID output", motorOutput);
    // armMotor.setVoltage(motorOutput);
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
