package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.chargedup.LoadingArea;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToLoadingZone extends CommandBase {
  private final DriveSubsystem drive;
  private final LoadingArea loadingArea = AutoConstants.loadingArea;
  private final LOADING_SIDE selectedLoadingSide;
  private Command currentCommand;

  public enum LOADING_SIDE {
    LEFT,
    RIGHT
  }

  public GoToLoadingZone(LOADING_SIDE selectedLoadingSide, DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    this.selectedLoadingSide = selectedLoadingSide;
    currentCommand = new InstantCommand();
  }

  public Command getCommand(LOADING_SIDE loadingPosition, Pose2d pose) {
    Command command;
    if (loadingArea.isPoseWithinScoringArea(pose)) {
      GoToPose goToPose;
      switch (loadingPosition) {
        case LEFT:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationLeft().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case RIGHT:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationRight().getPoseMeters(),
                  new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        default:
          command = new InstantCommand();
          break;
      }
    } else {
      command = new InstantCommand();
    }
    return command;
  }

  @Override
  public void initialize() {
    currentCommand = getCommand(selectedLoadingSide, drive.getPose());
    currentCommand.schedule();
  }

  @Override
  public void execute() {
    if (currentCommand.isFinished()) {
      currentCommand = getCommand(selectedLoadingSide, drive.getPose());
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
