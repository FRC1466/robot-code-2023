package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.util.HashMap;
import java.util.List;

public class PathBuilder {
  private SwerveAutoBuilder autoBuilder;

  public PathBuilder(SwerveSubsystem drivebase) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new PrintCommand("Passed intakedown 1"));

    autoBuilder =
        new SwerveAutoBuilder(
            drivebase::getPose, // Functional interface to feed supplier
            drivebase::resetOdometry,
            drivebase.getKinematics(),
            // Position controllers
            new PIDConstants(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I,
                AutoConstants.translationController.D),
            new PIDConstants(
                AutoConstants.thetaController.P,
                AutoConstants.thetaController.I,
                AutoConstants.thetaController.D),
            drivebase::setDesiredModuleStates,
            eventMap,
            true,
            drivebase);
  }

  public Command getSwerveCommand(List<PathPlannerTrajectory> path) {
    return autoBuilder.fullAuto(path);
  }
}
