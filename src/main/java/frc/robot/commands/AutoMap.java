package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.EndEffector;
import frc.robot.subsystems.manipulator.VirtualFourBar;
import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();

  public AutoMap(Superstructure superstructure, EndEffector effector, VirtualFourBar arm) {

    eventMap.put("ScoreLow", superstructure.scoreLow());
    eventMap.put("ScoreMid", superstructure.scoreMid());
    eventMap.put("ScoreHigh", superstructure.scoreHigh());
    eventMap.put("PickupGround", superstructure.pickupGround());
    eventMap.put("Store", superstructure.store());
    eventMap.put("Ground", arm.ground());
    eventMap.put("Loft", arm.loft());
    eventMap.put("Vertical", arm.vertical());
    eventMap.put("Mid", arm.mid());
    eventMap.put("High", arm.high());
    eventMap.put("Intake", effector.intake());
    eventMap.put("Launch", effector.launch());
    eventMap.put("PowerLaunch", effector.powerLaunch());
    eventMap.put("StopIntake", effector.stop());
  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}
