package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class ComplexAuto extends SequentialCommandGroup {
    public ComplexAuto(DriveSubsystem m_drive) {
        SwerveTrajectory m_swerveTraj = new SwerveTrajectory(m_drive);
        
        addCommands(
            m_swerveTraj.getCommand()
        );
    }
}
