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
  private HashMap<String, Command> eventMap = new HashMap<>();
  private HashMap<String, Supplier<Command>> eventMapGetter = new HashMap<>();
  public static final String ArmToGround = "ArmToGround",
      ArmToLoadingStation = "ArmToLoadingStation",
      ArmToStore = "ArmToStore",
      ArmToMid = "ArmToMid",
      ArmToVertical = "ArmToVertical",
      GrabCone = "GrabCone",
      GrabCube = "GrabCube",
      GrabEnsureNeutral = "GrabEnsureNeutral",
      GrabOpen = "GrabOpen",
      DropObjectAndStore = "DropObjectAndStore",
      ScoreArmLow = "ScoreArmLow",
      ScoreArmMid = "ScoreArmMid",
      PickupGroundReady = "PickupGroundReady",
      PickupLoadingStationReady = "PickupLoadingStationReady",
      PickupConeAndStore = "PickupConeAndStore",
      PickupCubeAndStore = "PickupCubeAndStore";

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

    eventMapGetter.put("GrabCone", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CONE)));
    eventMapGetter.put(
        "GrabEnsureNeutral",
        () ->
            Commands.runOnce(
                () -> {
                  if (gripper.getCurrentIntake() == INTAKE.OPEN) {
                    gripper.setGripper(INTAKE.STORE);
                  }
                }));
    eventMapGetter.put("GrabCube", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CUBE)));
    eventMapGetter.put("GrabOpen", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.OPEN)));

    /* Compositions */
    eventMapGetter.put(
        "DropObjectAndStore",
        () ->
            getCommandInMap(AutoMap.GrabOpen)
                .andThen(
                    Commands.parallel(
                        Commands.waitSeconds(0.2)
                            .andThen(getCommandInMap(AutoMap.GrabEnsureNeutral)),
                        getCommandInMap(AutoMap.ArmToStore))));

    eventMapGetter.put(
        "ScoreArmLow",
        () ->
            getCommandInMap(AutoMap.GrabEnsureNeutral)
                .andThen(getCommandInMap(AutoMap.ArmToGround)));

    eventMapGetter.put(
        "ScoreArmMid",
        () ->
            getCommandInMap(AutoMap.GrabEnsureNeutral).andThen(getCommandInMap(AutoMap.ArmToMid)));

    eventMapGetter.put(
        "PickupGroundReady",
        () ->
            getCommandInMap(AutoMap.GrabEnsureNeutral)
                .andThen(getCommandInMap(AutoMap.ArmToGround))
                .andThen(getCommandInMap(AutoMap.GrabOpen)));

    eventMapGetter.put(
        "PickupLoadingStationReady",
        () ->
            getCommandInMap(AutoMap.GrabEnsureNeutral)
                .andThen(getCommandInMap(AutoMap.ArmToLoadingStation))
                .andThen(getCommandInMap(AutoMap.GrabOpen)));

    eventMapGetter.put(
        "PickupConeAndStore",
        () ->
            getCommandInMap(AutoMap.GrabCone)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(getCommandInMap(AutoMap.ArmToStore)));

    eventMapGetter.put(
        "PickupCubeAndStore",
        () ->
            getCommandInMap(AutoMap.GrabCube)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(getCommandInMap(AutoMap.ArmToStore)));

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
