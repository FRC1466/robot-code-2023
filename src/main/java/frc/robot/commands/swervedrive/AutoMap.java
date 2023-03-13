package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Gripper.INTAKE;
import frc.robot.subsystems.manipulator.VirtualFourBar;
import frc.robot.subsystems.manipulator.VirtualFourBar.ARM;
import java.util.HashMap;
import java.util.function.Supplier;

public class AutoMap {
  private final HashMap<String, Command> eventMap = new HashMap<>();
  private final HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();
  public static final String ArmToGround = "ArmToGround",
      ArmToLoadingStation = "ArmToLoadingStation",
      ArmToStore = "ArmToStore",
      ArmToMid = "ArmToMid",
      ArmToVertical = "ArmToVertical",
      ObjectGrab = "ObjectGrab",
      ObjectDrop = "ObjectDrop",
      GripperOff = "GripperOff",
      DropObjectAndStore = "DropObjectAndStore",
      PickupGroundReady = "PickupGroundReady",
      PickupLoadingStationReady = "PickupLoadingStationReady";

  public AutoMap(Gripper gripper, VirtualFourBar arm) {

    /* Single setup */
    eventMapGetter.put(
        "ArmToGround",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.GROUND))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmToLoadingStation",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.STATION))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmToStore",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.STORAGE))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmToMid",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.MID))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));
    eventMapGetter.put(
        "ArmToVertical",
        () ->
            Commands.runOnce(() -> arm.setArm(ARM.VERTICAL))
                .andThen(Commands.waitUntil(() -> arm.isAtSetpoint())));

    eventMapGetter.put("ObjectGrab", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.IN)));
    eventMapGetter.put("ObjectDrop", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.OUT)));
    eventMapGetter.put("GripperOff", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.STOP)));

    /* Compositions */
    eventMapGetter.put(
        "DropObjectAndStore",
        () ->
            getCommandInMap(AutoMap.ObjectDrop)
                .andThen(
                    Commands.parallel(
                        Commands.waitSeconds(0.2).andThen(getCommandInMap(AutoMap.GripperOff)),
                        getCommandInMap(AutoMap.ArmToStore))));

    eventMapGetter.put(
        "PickupGroundReady",
        () -> getCommandInMap(AutoMap.ArmToGround).andThen(getCommandInMap(AutoMap.ObjectGrab)));

    eventMapGetter.put(
        "PickupLoadingStationReady",
        () ->
            getCommandInMap(AutoMap.ArmToLoadingStation)
                .andThen(getCommandInMap(AutoMap.ObjectGrab)));

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
