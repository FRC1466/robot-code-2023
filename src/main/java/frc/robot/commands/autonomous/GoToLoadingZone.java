package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.HolonomicPose2d;
import frc.lib.util.RectanglePoseArea;
import frc.lib.util.chargedup.LoadingArea;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToLoadingZone {
  private DriveSubsystem drive;
  private LoadingArea loadingArea;

  public GoToLoadingZone(DriveSubsystem drive) {
    this.drive = drive;
    loadingArea =
        new LoadingArea(
            new RectanglePoseArea(new Translation2d(9.91, 6.82), new Translation2d(16.24, 7.97)),
            new RectanglePoseArea(new Translation2d(13.24, 5.66), new Translation2d(16.51, 7.97)),
            new HolonomicPose2d(new Pose2d(15.79, 7.34, new Rotation2d()), new Rotation2d()),
            new HolonomicPose2d(new Pose2d(15.75, 6.00, new Rotation2d()), new Rotation2d()));
  }

  public Command getCommand(int loadingPosition, Pose2d pose) {
    Command command;
    if (loadingArea.isPoseWithinScoringArea(pose)) {
      GoToPose goToPose;
      switch (loadingPosition) {
        case 1:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationLeft().getPathPoint(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          break;
        case 2:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationRight().getPathPoint(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
        default:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationRight().getPathPoint(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          break;
      }
      command = goToPose.getCommand();
    } else {
      command = new InstantCommand();
    }
    return command;
  }
}
