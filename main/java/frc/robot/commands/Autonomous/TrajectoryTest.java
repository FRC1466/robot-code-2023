package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryTest extends CommandBase {
    private final DriveSubsystem m_drive;
    private Timer time = new Timer();

    private RamseteController m_rcontroller = new RamseteController();
    private TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_SPEED_MPS, AutoConstants.MAX_ACCELERATION_MPS).setKinematics(DriveConstants.KINEMATICS);
    private Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
      );
    
    /**
     * Testing Trajectory for swerve using ramsete
     * @param subsystem drive subsystem
     */
    TrajectoryTest(DriveSubsystem subsystem) {
        m_drive = subsystem;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        time.start();
    }

    @Override
    public void execute() {
        m_drive.updateRobotPose();
        m_drive.updateSpeedsTrajectory(time.get(), m_rcontroller, exampleTrajectory);
        m_drive.updateModuleStates();
        m_drive.drive();
    }

    @Override
    public boolean isFinished() {
        if(time.hasElapsed(exampleTrajectory.getTotalTimeSeconds())) {
            time.stop();
            return true;
        } else {
            return false;
        }
    }
}
