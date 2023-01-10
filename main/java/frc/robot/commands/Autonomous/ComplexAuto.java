package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

public class ComplexAuto extends SequentialCommandGroup {
    public ComplexAuto(DriveSubsystem m_drive, TelemetrySubsystem m_tele) {
        
        addCommands(
            new InstantCommand(() -> m_tele.updatePIDConstants()),
            new SwerveTrajectory(m_drive).getCommand()
        );
    }
}
