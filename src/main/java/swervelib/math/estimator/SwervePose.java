package swervelib.math.estimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import java.util.Optional;
import swervelib.math.SwerveModuleState2;

public abstract class SwervePose {

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public abstract void setVisionMeasurementStdDevs(Matrix<N6, N1> visionMeasurementStdDevs);

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters The position on the field that your robot is at.
   */
  public abstract void resetPosition(
      Rotation3d gyroAngle, SwerveModulePosition[] modulePositions, Pose3d poseMeters);

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public abstract Pose3d getPoseMeters();

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwervePose#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link
   *     SwervePose#updateWithTime(double,Rotation2d,double,double)} then you must use a timestamp
   *     with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same epoch as
   *     {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that you should use
   *     {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source or sync the
   *     epochs.
   */
  public abstract void addVisionMeasurement(Pose3d visionRobotPoseMeters, double timestampSeconds);

  public abstract void addVisionMeasurement(
      Pose3d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N6, N1> visionMeasurementStdDevs);

  public abstract Pose3d update(
      Rotation3d gyroAngle,
      Optional<Translation3d> accel,
      SwerveModuleState2[] moduleStates,
      SwerveModulePosition[] modulePositions);

  public abstract Pose3d updateWithTime(
      double currentTimeSeconds,
      Rotation3d gyroAngle,
      Optional<Translation3d> accel,
      SwerveModuleState2[] moduleStates,
      SwerveModulePosition[] modulePositions);
}
