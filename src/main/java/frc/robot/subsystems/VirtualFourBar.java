package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.ArmConstants;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private Encoder armEncoder;
  private PIDController armPID;

  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    armEncoder = new Encoder(0, 1);
    armEncoder.setDistancePerPulse(1 / 256);

    armPID =
        new PIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
  }

  private void configArmMotor() {
    armMotor.configFactoryDefault();
    armMotor.configAllSettings(Robot.moduleConfigs.armConfig);
  }

  public void setArm(double a) {
    var motorOutput = armPID.calculate(armEncoder.getDistance(), a);
    armMotor.setVoltage(motorOutput);
  }
}
