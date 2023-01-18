package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.AdjustableTelemetry;

public class ComplexAuto extends SequentialCommandGroup {
    private Command swerveControllerCommand;
    private List<PathPlannerTrajectory> path;

    /**
     * create new complex auto
     * @param drive drive subsystem
     * @param tele telemetry
     */
    public ComplexAuto(DriveSubsystem drive, AdjustableTelemetry tele) {
        path = PathPlanner.loadPathGroup("Test Path", 
            new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("intakeDown", new PrintCommand("Passed intakedown 1"));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
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
        
        swerveControllerCommand = autoBuilder.fullAuto(path);

        
        addCommands(
            new InstantCommand(() -> tele.updatePIDConstants()),
            swerveControllerCommand
        );
    }
}
