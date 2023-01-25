package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.constants.RobotConstants.Swerve;
import frc.robot.subsystems.DriveSubsystem;

public class PathBuilder {
    private SwerveAutoBuilder autoBuilder;
    
    public PathBuilder(DriveSubsystem drive) {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("intakeDown", new PrintCommand("Passed intakedown 1"));

        autoBuilder = new SwerveAutoBuilder(
            drive::getPose, // Functional interface to feed supplier
            drive::resetPose,
            Swerve.KINEMATICS,
            // Position controllers
            new PIDConstants(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I, 
                AutoConstants.translationController.D),
            new PIDConstants(
                AutoConstants.thetaController.P,
                AutoConstants.thetaController.I, 
                AutoConstants.thetaController.D),
            drive::setDesiredModuleStates,
            eventMap,
            true,
            drive);
    }

    public Command getSwerveCommand(List<PathPlannerTrajectory> path) {
        return autoBuilder.fullAuto(path);
    }
}
