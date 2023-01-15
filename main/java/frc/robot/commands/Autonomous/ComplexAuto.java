package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.AdjustableTelemetry;

public class ComplexAuto extends SequentialCommandGroup {
    private SwerveTrajectory swerveTrajectory;

    /**
     * create new complex auto
     * @param drive drive subsystem
     * @param tele telemetry
     */
    public ComplexAuto(DriveSubsystem drive, AdjustableTelemetry tele) {
        swerveTrajectory = new SwerveTrajectory(drive, PathPlanner.loadPath("Test Path", 
        new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS)));
        
        addCommands(
            new InstantCommand(() -> tele.updatePIDConstants()),
            swerveTrajectory.getCommand()
        );
    }
}
