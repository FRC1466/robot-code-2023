package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private Spark ledSpark;

  public LED() {
    ledSpark = new Spark(LEDConstants.PWMPort);
  }

  public void setColor() {
    ledSpark.set(DriverStation.getAlliance() == Alliance.Blue ? 0.87 : 0.61);
  }
}
