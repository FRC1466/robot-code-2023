package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.constants.RobotConstants.Swerve;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GoToPose {
  PPSwerveControllerCommand ppSwerveCommand;
  PathPlannerTrajectory traj;

  public GoToPose(
      Pose2d pose, Rotation2d heading, PathConstraints constraints, DriveSubsystem drive) {
    Translation2d translation = pose.getTranslation();
    Rotation2d holonomic = pose.getRotation();

    PathPoint currentPathPoint =
        PathPoint.fromCurrentHolonomicState(drive.getPose(), drive.getCurrentChassisSpeeds());

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            add(new PathPoint(translation, heading, holonomic));
          }
        };

    traj =
        PathPlanner.generatePath(
            constraints, path); // position, heading(direction of travel), holonomic rotation

    ppSwerveCommand =
        new PPSwerveControllerCommand(
            traj,
            drive::getPose, // Pose supplier
            Swerve.KINEMATICS, // SwerveDriveKinematics
            new PIDController(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I,
                AutoConstants.translationController
                    .D), // X controller. Tune these values for your robot. Leaving them 0 will only
            // use feedforwards.
            new PIDController(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I,
                AutoConstants.translationController
                    .D), // Y controller (usually the same values as X controller)
            new PIDController(
                AutoConstants.thetaController.P,
                AutoConstants.thetaController.I,
                AutoConstants.thetaController
                    .D), // Rotation controller. Tune these values for your robot. Leaving them 0
            // will only use feedforwards.
            drive::setDesiredModuleStates, // Module states consumer
            true,
            drive // Requires this drive subsystem
            );
  }

  public GoToPose(PathPoint pathPoint, PathConstraints constraints, DriveSubsystem drive) {

    PathPoint currentPathPoint =
        PathPoint.fromCurrentHolonomicState(drive.getPose(), drive.getCurrentChassisSpeeds());

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            add(pathPoint);
          }
        };

    traj =
        PathPlanner.generatePath(
            constraints, path); // position, heading(direction of travel), holonomic rotation

    ppSwerveCommand =
        new PPSwerveControllerCommand(
            traj,
            drive::getPose, // Pose supplier
            Swerve.KINEMATICS, // SwerveDriveKinematics
            new PIDController(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I,
                AutoConstants.translationController
                    .D), // X controller. Tune these values for your robot. Leaving them 0 will only
            // use feedforwards.
            new PIDController(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I,
                AutoConstants.translationController
                    .D), // Y controller (usually the same values as X controller)
            new PIDController(
                AutoConstants.thetaController.P,
                AutoConstants.thetaController.I,
                AutoConstants.thetaController
                    .D), // Rotation controller. Tune these values for your robot. Leaving them 0
            // will only use feedforwards.
            drive::setDesiredModuleStates, // Module states consumer
            true,
            drive // Requires this drive subsystem
            );
  }

  public PPSwerveControllerCommand getCommand() {
    return ppSwerveCommand;
  }
}
