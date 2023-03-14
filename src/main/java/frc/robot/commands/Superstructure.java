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
    return arm.ground().andThen(Commands.parallel(effector.intake(), arm.hold()));
  }

  public Command pickupStation() {
    return arm.station().andThen(Commands.parallel(effector.intake(), arm.hold()));
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

  public Command launchStore() {
    return effector.launch().andThen(arm.store());
  }

  public Command scoreLow() {
    return arm.ground().andThen(effector.launch());
  }

  public Command scoreMid() {
    return arm.mid().andThen(effector.drop());
  }

  public Command scoreHigh() {
    return arm.high().andThen(effector.launch());
  }
}
