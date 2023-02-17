package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import webblib.util.chargedup.ScoringArea;

public class GoToScoring extends CommandBase {
  private final SwerveSubsystem drive;
  private final List<ScoringArea> scoreAreaList = Auton.scoreAreaList;
  private final POSITION selectedPosition;
  private Command currentCommand;

  public enum POSITION {
    LEFT,
    MIDDLE,
    RIGHT
  }

  public GoToScoring(SwerveSubsystem drive, POSITION selectedPosition) {
    this.drive = drive;
    addRequirements(drive);
    this.selectedPosition = selectedPosition;
    this.currentCommand = new InstantCommand();
  }

  /**
   * Get best scoring area. Assumes scoring area zones do not overlap.
   *
   * @param pose current pose of robot
   * @return either null if not in scoring area, or the scoring are if in scoring area
   */
  private Optional<ScoringArea> getBestScoringArea(Pose2d pose) {
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
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case MIDDLE:
          goToPose =
              new GoToPose(
                  scoringArea.get().getMiddlePosition().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case RIGHT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getRightPosition().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        default:
          System.out.println(scorePosition.toString());
          throw new IllegalArgumentException("Unsupported enum");
      }
    } else {
      command = new InstantCommand();
    }
    return command;
  }

  @Override
  public void initialize() {
    currentCommand = getCommand(selectedPosition, drive.getPose());
    currentCommand.schedule();
  }

  @Override
  public void execute() {
    if (currentCommand.isFinished()) {
      currentCommand = getCommand(selectedPosition, drive.getPose());
      currentCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      currentCommand.cancel();
    }
  }
}
