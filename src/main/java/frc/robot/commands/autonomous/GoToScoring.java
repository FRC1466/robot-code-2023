package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.HolonomicPose2d;
import frc.lib.util.RectanglePoseArea;
import frc.lib.util.chargedup.ScoringArea;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class GoToScoring {
  private DriveSubsystem drive;
  private List<ScoringArea> scoreAreaList;

  public enum POSITION {
    RIGHT,
    MIDDLE,
    LEFT
  }

  public GoToScoring(DriveSubsystem drive) {
    this.drive = drive;
    scoreAreaList =
        new ArrayList<>() {
          {
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)),
                    // diagonal y's should not overlap
                    new HolonomicPose2d(new Pose2d(1.62, 4.95, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 4.40, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 3.84, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
                    new HolonomicPose2d(new Pose2d(1.62, 3.30, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 2.72, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 2.19, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
                    new HolonomicPose2d(new Pose2d(1.62, 1.61, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 1.03, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 0.55, new Rotation2d()), new Rotation2d())));
          }
        };
  }

  /**
   * @param pose current pose of robot
   * @return either null if not in scoring area, or the scoring are if in scoring area
   */
  private Optional<ScoringArea> getBestScoringArea(Pose2d pose) { // do an optional
    ScoringArea bestArea = null;
    for (ScoringArea area : scoreAreaList) {
      if (area.isPoseWithinScoringArea(pose)) bestArea = area;
    }
    if (bestArea == null) {
      return Optional.empty();
    }
    return Optional.of(bestArea);
  }

  public Command getCommand(POSITION scorePosition, Pose2d pose) {
    System.out.println("Scoring Position Scheduled");
    Optional<ScoringArea> scoringArea = getBestScoringArea(pose);
    Command command;
    if (!scoringArea.isEmpty()) {
      GoToPose goToPose;
      switch (scorePosition) {
        case LEFT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getLeftPosition().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case MIDDLE:
          goToPose =
              new GoToPose(
                  scoringArea.get().getMiddlePosition().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
        case RIGHT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getRightPosition().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
        default:
          throw new IllegalArgumentException("Unsupported enum");
      }
    } else {
      command = new InstantCommand();
    }
    return command;
  }
}
