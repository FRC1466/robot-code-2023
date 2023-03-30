package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.EndEffector;
import frc.robot.subsystems.manipulator.VirtualFourBar;

public class Superstructure {
  private final EndEffector effector;
  private final VirtualFourBar arm;

  public Superstructure(EndEffector effector, VirtualFourBar arm) {
    this.effector = effector;
    this.arm = arm;
  }

  public Command pickupGround() {
    return arm.ground().andThen(effector.intake());
  }

  public Command pickupStation() {
    return arm.station().andThen(effector.intake());
  }

  public Command store() {
    return arm.store();
  }

  public Command vertical() {
    return arm.vertical();
  }

  public Command dropStore() {
    return effector.drop().andThen(arm.store());
  }

  public Command dropMidStore() {
    return arm.midScore().andThen(effector.drop().andThen(arm.store()));
  }

  public Command launchStore() {
    return effector.launch().andThen(arm.store());
  }

  public Command scoreLowLaunch() {
    return arm.storeLaunchReady().andThen(launchStore());
  }

  public Command scoreLow() {
    return arm.ground().andThen(effector.launch());
  }

  public Command scoreMid() {
    return arm.mid().andThen(effector.launch());
  }

  public Command scoreHigh() {
    return arm.high().andThen(effector.launch());
  }

  public Command launchConeToHigh() {
    return Commands.parallel(arm.store(), Commands.waitSeconds(0.2).andThen(effector.drop()));
  }

  public Command scoreConeHigh() {
    return arm.highLaunchReady().andThen(launchConeToHigh());
  }
}
