package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import webblib.util.chargedup.LoadingArea;

public class GoToLoadingZone extends CommandBase {
  private final SwerveSubsystem drive;
  private final LoadingArea loadingArea = Auton.loadingArea;
  private final LOADING_SIDE selectedLoadingSide;
  private Command currentCommand;

  public enum LOADING_SIDE {
    LEFT,
    RIGHT
  }

  public GoToLoadingZone(LOADING_SIDE selectedLoadingSide, SwerveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    this.selectedLoadingSide = selectedLoadingSide;
    currentCommand = Commands.none();
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
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case RIGHT:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationRight().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else {
      command = Commands.none();
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
    currentCommand.cancel();
  }
}
