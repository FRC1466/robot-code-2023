package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.chargedup.ScoringArea;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import java.util.Optional;

public class GoToScoring extends CommandBase {
  private DriveSubsystem drive;
  private List<ScoringArea> scoreAreaList = AutoConstants.scoreAreaList;
  private Command currentCommand;

  public enum POSITION {
    LEFT,
    MIDDLE,
    RIGHT
  }

  private POSITION selectedPosition;

  public GoToScoring(DriveSubsystem drive, POSITION selectedPosition) {
    this.drive = drive;
    addRequirements(drive);
    this.selectedPosition = selectedPosition;
    this.currentCommand = new InstantCommand();
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
          break;
        case RIGHT:
          goToPose =
              new GoToPose(
                  scoringArea.get().getRightPosition().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
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
}
