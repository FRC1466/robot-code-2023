package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.ArmConstants;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder armEncoder;
  private PIDController armPID;

  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    armEncoder = new DutyCycleEncoder(0);
    armEncoder.setDistancePerRotation(1);
    armEncoder.setPositionOffset(0);

    armPID =
        new PIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
  }

  private void configArmMotor() {
    armMotor.configFactoryDefault();
    armMotor.configAllSettings(Robot.moduleConfigs.armConfig);
  }

  public void setArm(double a) {
    SmartDashboard.putNumber("setpoint", a);
    var motorOutput = MathUtil.clamp(armPID.calculate(armEncoder.getDistance(), a), -12, 12);
    armMotor.setVoltage(motorOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(armEncoder);
    SmartDashboard.putNumber("abs arm", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("norm arm", armEncoder.getDistance());
  }
}
