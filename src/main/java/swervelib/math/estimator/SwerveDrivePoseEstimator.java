package swervelib.math.estimator;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import java.util.NoSuchElementException;
import swervelib.math.SwerveKinematics2;
import swervelib.math.SwerveModuleState2;

/**
 * This class wraps {@link SwerveDriveOdometry Swerve Drive Odometry} to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is intended to be a
 * drop-in replacement for {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 */
public class SwerveDrivePoseEstimator {
  private final SwerveDriveOdometry m_odometry;
  private final Matrix<N4, N1> m_q = new Matrix<>(Nat.N4(), Nat.N1());
  private final int m_numModules;
  private final Matrix<N4, N4> m_visionK = new Matrix<>(Nat.N4(), Nat.N4());

  private static final double kBufferDuration = 1.5;

  private final TimeInterpolatableBuffer<Pose3d> m_poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDuration);

  private Pose3dFix m_poseEstimate;

  /**
   * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision
   * measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
   * meters for x, 0.9 meters for y, and 0.9 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   */
  public SwerveDrivePoseEstimator(
      SwerveKinematics2 kinematics,
      Rotation2d gyroAngle,
      SwerveModuleState2[] modulePositions,
      Pose2d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPoseMeters,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9));
  }

  /**
   * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision
   * measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
   * meters for x, 0.9 meters for y, and 0.9 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   */
  public SwerveDrivePoseEstimator(
      SwerveKinematics2 kinematics,
      Rotation3d gyroAngle,
      SwerveModuleState2[] modulePositions,
      Pose3d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPoseMeters,
        VecBuilder.fill(0.1, 0.1, 0.1, 0.1),
        VecBuilder.fill(1.0, 1.0, 1.0, 2.0));
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public SwerveDrivePoseEstimator(
      SwerveKinematics2 kinematics,
      Rotation3d gyroAngle,
      SwerveModuleState2[] modulePositions,
      Pose3d initialPoseMeters,
      Matrix<N4, N1> stateStdDevs,
      Matrix<N4, N1> visionMeasurementStdDevs) {
    m_odometry =
        new SwerveDriveOdometry(
            kinematics,
            gyroAngle,
            modulePositions,
            new Pose3dFix(initialPoseMeters.getTranslation(), initialPoseMeters.getRotation()));
    m_poseEstimate = m_odometry.getPoseMeters3d();

    for (int i = 0; i < 4; ++i) {
      m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }

    m_numModules = modulePositions.length;

    setVisionMeasurementStdDevs3d(visionMeasurementStdDevs);
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public SwerveDrivePoseEstimator(
      SwerveKinematics2 kinematics,
      Rotation2d gyroAngle,
      SwerveModuleState2[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this(
        kinematics,
        new Rotation3d(0, 0, gyroAngle.getRadians()),
        modulePositions,
        new Pose3d(initialPoseMeters),
        VecBuilder.fill(stateStdDevs.get(0, 0), stateStdDevs.get(1, 0), 0, stateStdDevs.get(2, 0)),
        VecBuilder.fill(
            visionMeasurementStdDevs.get(0, 0),
            visionMeasurementStdDevs.get(1, 0),
            0,
            visionMeasurementStdDevs.get(2, 0)));
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    setVisionMeasurementStdDevs3d(
        VecBuilder.fill(
            visionMeasurementStdDevs.get(0, 0),
            visionMeasurementStdDevs.get(1, 0),
            0,
            visionMeasurementStdDevs.get(2, 0)));
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs3d(Matrix<N4, N1> visionMeasurementStdDevs) {
    var r = new double[4];
    for (int i = 0; i < 4; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 4; ++row) {
      if (m_q.get(row, 0) == 0.0) {
        m_visionK.set(row, row, 0.0);
      } else {
        m_visionK.set(
            row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
      }
    }
  }

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
  public void resetPosition(
      Rotation3d gyroAngle, SwerveModuleState2[] modulePositions, Pose3d poseMeters) {
    // Reset state estimate and error covariance
    m_odometry.resetPosition(
        gyroAngle,
        modulePositions,
        new Pose3dFix(poseMeters.getTranslation(), poseMeters.getRotation()));
    m_poseBuffer.clear();
    m_poseEstimate = m_odometry.getPoseMeters3d();
  }

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
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModuleState2[] modulePositions, Pose2d poseMeters) {
    resetPosition(
        new Rotation3d(0, 0, gyroAngle.getRadians()), modulePositions, new Pose3d(poseMeters));
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return m_poseEstimate.toPose2d();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose3dFix getEstimatedPosition3d() {
    return m_poseEstimate;
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime}
   *     then you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
   *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
   *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
   *     your time source or sync the epochs.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    addVisionMeasurement(new Pose3dFix(visionRobotPoseMeters), timestampSeconds);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime}
   *     then you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
   *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
   *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
   *     your time source or sync the epochs.
   */
  public void addVisionMeasurement(Pose3d visionRobotPoseMeters, double timestampSeconds) {
    // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
    visionRobotPoseMeters =
        new Pose3dFix(visionRobotPoseMeters.getTranslation(), visionRobotPoseMeters.getRotation());
    try {
      if (m_poseBuffer.getInternalBuffer().lastKey() - kBufferDuration > timestampSeconds) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Step 1: Get the pose odometry measured at the moment the vision measurement was made.
    var sample = m_poseBuffer.getSample(timestampSeconds);

    if (sample.isEmpty()) {
      return;
    }

    var record = sample.get();

    var odometry_backtrack = m_odometry.getPoseMeters3d().log(record);
    // System.out.println("ODO BACKTACK " + odometry_backtrack);

    var odometry_fastforward =
        new Twist3d(
            -odometry_backtrack.dx,
            -odometry_backtrack.dy,
            -odometry_backtrack.dz,
            -odometry_backtrack.rx,
            -odometry_backtrack.ry,
            -odometry_backtrack.rz);

    var old_estimate = m_poseEstimate.exp(odometry_backtrack);

    // Step 2: Measure the twist between the odometry pose and the vision pose.
    var twist = old_estimate.log(visionRobotPoseMeters);

    // Step 3: We should not trust the twist entirely, so instead we scale this twist by a Kalman
    // gain matrix representing how much we trust vision measurements compared to our current pose.
    var twist_rvec = VecBuilder.fill(twist.rx, twist.ry, twist.rz);
    var twist_angle = twist_rvec.norm();
    var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dz, twist_angle));

    Twist3d scaledTwist;
    var rx = twist_rvec.get(0, 0) / twist_angle * k_times_twist.get(3, 0);
    var ry = twist_rvec.get(1, 0) / twist_angle * k_times_twist.get(3, 0);
    var rz = twist_rvec.get(2, 0) / twist_angle * k_times_twist.get(3, 0);

    // Step 4: Convert back to Twist3d.
    if (!((Double) rx).isNaN() || !((Double) ry).isNaN() || !((Double) rz).isNaN()) {
      scaledTwist =
          new Twist3d(
              k_times_twist.get(0, 0),
              k_times_twist.get(1, 0),
              k_times_twist.get(2, 0),
              rx,
              ry,
              rz);
    } else {
      scaledTwist =
          new Twist3d(
              k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0), 0, 0, 0);
    }

    // System.out.println("OLD ESTIMATE" + scaledTwist);
    old_estimate = old_estimate.exp(scaledTwist);

    m_poseEstimate = old_estimate.exp(odometry_fastforward);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime},
   *     then you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
   *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
   *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
   *     your time source in this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public void addVisionMeasurement(
      Pose3d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N4, N1> visionMeasurementStdDevs) {
    setVisionMeasurementStdDevs3d(visionMeasurementStdDevs);
    addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime},
   *     then you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
   *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
   *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
   *     your time source in this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    addVisionMeasurement(new Pose3d(visionRobotPoseMeters), timestampSeconds);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose3dFix update(Rotation3d gyroAngle, SwerveModuleState2[] modulePositions) {
    return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, modulePositions);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(Rotation2d gyroAngle, SwerveModuleState2[] modulePositions) {
    return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, modulePositions);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState2[] modulePositions) {
    return updateWithTime(
            currentTimeSeconds, new Rotation3d(0, 0, gyroAngle.getRadians()), modulePositions)
        .toPose2d();
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose3dFix updateWithTime(
      double currentTimeSeconds, Rotation3d gyroAngle, SwerveModuleState2[] modulePositions) {
    if (modulePositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    var lastOdom = m_odometry.getPoseMeters3d();
    var currOdom = m_odometry.update(gyroAngle, modulePositions);
    m_poseBuffer.addSample(currentTimeSeconds, currOdom);

    m_poseEstimate = m_poseEstimate.exp(lastOdom.log(currOdom));
    // var a = new Pose3dFix(new Translation3d(3.75, 2.95, 0.00), new Rotation3d(new
    // Quaternion(0.9999999999999793, 0.0, 0.0, 2.0352360299846772E-7)));
    // var b = new Pose3dFix(new Translation3d(3.66, 2.93, 0.00), new Rotation3d(new
    // Quaternion(0.9999999999999782, 0.0, 0.0, 2.0859477994905617E-7)));
    // System.out.println(a.log(b));
    // System.out.println(lastOdom.log(currOdom).toString());
    // System.out.println(m_poseEstimate.exp(lastOdom.log(currOdom)).toString());

    if (((Double) m_poseEstimate.getX()).isNaN()) {
      throw new IllegalArgumentException("no");
    }
    return getEstimatedPosition3d();
  }
}
