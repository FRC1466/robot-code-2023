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
      FreeCone = "FreeCone",
      FreeCube = "FreeCube",
      GripperOff = "GripperOff",
      DropObjectAndStore = "DropObjectAndStore",
      EnsureGrabAndStore = "EnsureGrabAndStore",
      EnsureGrabAndVertical = "EnsureGrabAndVertical",
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

    eventMapGetter.put("GrabCone", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CONEIN)));
    eventMapGetter.put("GrabCube", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CUBEIN)));
    eventMapGetter.put("FreeCone", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CONEOUT)));
    eventMapGetter.put("FreeCube", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.CUBEOUT)));
    eventMapGetter.put("GripperOff", () -> Commands.runOnce(() -> gripper.setGripper(INTAKE.STOP)));

    /* Compositions */
    eventMapGetter.put(
        "DropConeAndStore",
        () ->
            getCommandInMap(AutoMap.FreeCone)
                .andThen(
                    Commands.parallel(
                        Commands.waitSeconds(0.2),
                        getCommandInMap(AutoMap.ArmToStore))));
    eventMapGetter.put(
        "DropCubeAndStore",
        () ->
            getCommandInMap(AutoMap.FreeCube)
                .andThen(
                    Commands.parallel(
                         Commands.waitSeconds(0.2),
                        getCommandInMap(AutoMap.ArmToStore))));
    eventMapGetter.put(
        "PickupGroundReady",
        () ->
            getCommandInMap(AutoMap.ArmToGround)
                .andThen(getCommandInMap(AutoMap.GripperOff)));

    eventMapGetter.put(
        "PickupLoadingStationReady",
        () ->
            getCommandInMap(AutoMap.ArmToLoadingStation)
                .andThen(getCommandInMap(AutoMap.GripperOff)));

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
