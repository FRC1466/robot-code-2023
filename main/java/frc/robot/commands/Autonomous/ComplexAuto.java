package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AdjustableTelemetry;

public class ComplexAuto extends SequentialCommandGroup {
    private SwerveTrajectory st;
    public ComplexAuto(DriveSubsystem drive, AdjustableTelemetry tele) {
        st = new SwerveTrajectory(drive, PathPlanner.loadPath("Test Path", new PathConstraints(4, 3)));
        
        addCommands(
            new InstantCommand(() -> tele.updatePIDConstants()),
            st.getCommand()
        );
    }
}
