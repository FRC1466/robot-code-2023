package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.AdjustableTelemetry;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class ComplexAuto extends SequentialCommandGroup {
  private Command swerveControllerCommand;
  private List<PathPlannerTrajectory> path;

  /**
   * create new complex auto
   *
   * @param drive drive subsystem
   * @param tele telemetry
   */
  public ComplexAuto(DriveSubsystem drive, AdjustableTelemetry tele, PathBuilder builder) {
    path =
        PathPlanner.loadPathGroup(
            "Test Path",
            new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS));
    swerveControllerCommand = builder.getSwerveCommand(path);

    addCommands(new InstantCommand(() -> tele.updatePIDConstants()), swerveControllerCommand);
  }
}
