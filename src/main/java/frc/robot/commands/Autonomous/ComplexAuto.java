package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class ComplexAuto extends SequentialCommandGroup {
    public ComplexAuto(DriveSubsystem m_drive) {
        addCommands(
            new TrajectoryTest(m_drive)
        );
    }
}
