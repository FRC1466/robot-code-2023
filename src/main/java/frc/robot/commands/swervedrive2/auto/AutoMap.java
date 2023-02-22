package frc.robot.commands.swervedrive2.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Gripper.INTAKE;
import frc.robot.subsystems.manipulator.VirtualFourBar;
import frc.robot.subsystems.manipulator.VirtualFourBar.ARM;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoMap {
  private HashMap<String, Command> eventMap = new HashMap<>();
  private HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();

  public AutoMap(Gripper gripper, VirtualFourBar arm) {

    eventMapGetter.put(
        "ArmGround",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.GROUND))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmLoadingStation",
        () -> Commands.run(() -> arm.setArm(ARM.STATION), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmStoreObject",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.STORAGE))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmMidScore",
        () -> Commands.run(() -> arm.setArm(ARM.MID), arm).until(() -> arm.isAtSetpoint()));
    eventMapGetter.put(
        "ArmVertical",
        () -> Commands.run(() -> arm.setArm(ARM.VERTICAL), arm).until(() -> arm.isAtSetpoint()));

    eventMapGetter.put("ConeGrab", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CONE)));
    eventMapGetter.put(
        "EnsureNeutralGrab",
        () ->
            Commands.runOnce(
                () -> {
                  if (gripper.getCurrentIntake() == INTAKE.OPEN) {
                    gripper.setGripper(INTAKE.STORE);
                  }
                }));
    eventMapGetter.put("CubeGrab", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CUBE)));
    eventMapGetter.put("OpenGrab", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.OPEN)));

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
