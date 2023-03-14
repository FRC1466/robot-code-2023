package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PDHConstants;

public class PDH extends SubsystemBase {
  private final PowerDistribution power = new PowerDistribution(PDHConstants.port, ModuleType.kRev);

  public PDH() {
    power.setSwitchableChannel(false);
  }

  public void setSwitchableChannel(boolean isSwitchedOn) {
    power.setSwitchableChannel(isSwitchedOn);
  }

  public Command switchableOff() {
    return runOnce(() -> setSwitchableChannel(false));
  }

  public Command switchableOn() {
    return runOnce(() -> setSwitchableChannel(true));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("PDH Current", power.getTotalCurrent());
  }
}
