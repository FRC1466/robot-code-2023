package frc.robot.commands.swervedrive2.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Gripper.INTAKE;
import frc.robot.subsystems.manipulator.VirtualFourBar;
import frc.robot.subsystems.manipulator.VirtualFourBar.HEIGHT;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoMap {
  private HashMap<String, Command> eventMap = new HashMap<>();
  private HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();

  public AutoMap(Gripper gripper, VirtualFourBar arm) {

    eventMapGetter.put(
        "ArmGround",
        () -> Commands.run(() -> arm.setArm(HEIGHT.GROUND), arm));
    eventMapGetter.put(
        "ArmLoadingStation",
        () -> Commands.run(() -> arm.setArm(HEIGHT.STATION), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmStoreObject",
        () -> Commands.run(() -> arm.setArm(HEIGHT.STORAGE), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmMidScore",
        () -> Commands.run(() -> arm.setArm(HEIGHT.MID), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmStraightUp",
        () ->
            Commands.run(() -> arm.setArm(HEIGHT.STRAIGHT_UP), arm)
                .until(() -> arm.isAtSetpoint()));

    eventMapGetter.put(
        "ConeGrab",
        () ->
            Commands.runOnce(
                () ->
                    gripper.setDefaultCommand(
                        Commands.run(() -> gripper.setGripper(INTAKE.CONE), gripper))));
    eventMapGetter.put(
        "NeutralGrab",
        () ->
            Commands.runOnce(
                () ->
                    gripper.setDefaultCommand(
                        Commands.run(() -> gripper.setGripper(INTAKE.STORE), gripper))));
    eventMapGetter.put(
        "CubeGrab",
        () ->
            Commands.runOnce(
                () ->
                    gripper.setDefaultCommand(
                        Commands.run(() -> gripper.setGripper(INTAKE.CUBE), gripper))));
    eventMapGetter.put(
        "OpenGrab",
        () ->
            Commands.runOnce(
                () ->
                    gripper.setDefaultCommand(
                        Commands.run(() -> gripper.setGripper(INTAKE.OPEN), gripper))));

    eventMapGetter.forEach(
        (key, val) -> {
          eventMap.put(key, val.get());
        });
  }

  public HashMap<String, Command> getMap() {
    return eventMap;
  }

  public Command getCommandInMap(String key) {
    return eventMapGetter.get(key).get();
  }
}
