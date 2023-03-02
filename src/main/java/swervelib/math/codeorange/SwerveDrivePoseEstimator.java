// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib.math.codeorange;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.util.WPIUtilJNI;

import java.util.Optional;
import java.util.function.BiConsumer;
import swervelib.math.SwerveKinematics2;
import swervelib.math.SwerveModuleState2;


/**
 * This class wraps an {@link UnscentedKalmanFilter Unscented Kalman Filter} to fuse latency-compensated vision
 * measurements with swerve drive encoder velocity measurements. It will correct for noisy measurements and encoder
 * drift. It is intended to be an easy but more accurate drop-in for
 * {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop. If your loops are
 * faster or slower than the default of 0.02s, then you should change the nominal delta time using the secondary
 * constructor:
 * {@link SwerveDrivePoseEstimator#SwerveDrivePoseEstimator(Rotation3d, Pose3d, SwerveKinematics2, Matrix, Matrix,
 * Matrix, double)}.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave mostly like regular encoder odometry.
 *
 * <p>Our state-space system is:
 *
 * <p><strong> x = [[x, y, theta]]ᵀ </strong> in the field-coordinate system.
 *
 * <p><strong> u = [[vx, vy, omega]]ᵀ </strong> in the field-coordinate system.
 *
 * <p><strong> y = [[x, y, theta]]ᵀ </strong> in field coords from vision, or <strong> y =
 * [[theta]]ᵀ </strong> from the gyro.
 */
public class SwerveDrivePoseEstimator
{

  private final UnscentedKalmanFilter<N6, N6, N6>          m_observer;
  private final UnscentedKalmanFilter<N1, N1, N1>          m_accelObserver;
  private final SwerveKinematics2                          m_kinematics;
  private final BiConsumer<Matrix<N6, N1>, Matrix<N6, N1>> m_visionCorrect;
  private final KalmanFilterLatencyCompensator<N6, N6, N6> m_latencyCompensator;

  private final double m_nominalDt; // Seconds
  private       double m_prevTimeSeconds = -1.0;

  private Rotation3d m_gyroOffset;
  private Rotation3d m_previousAngle;
  private double m_previousRoll = 0;
  private double m_previousPitch = 0;
  private double m_previousYaw = 0;
  private SwerveModulePosition[] m_previousModulePositions;

  private Matrix<N6, N6> m_visionContR;

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to trust your model's
   *                                 state estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
   *                                 meters and radians.
   * @param localMeasurementStdDevs  Standard deviations of the encoder and gyro measurements. Increase these numbers to
   *                                 trust sensor readings from encoders and gyros less. This matrix is in the form
   *                                 [theta], with units in radians.
   * @param accelStateStdDevs        Standard deviations of the z axis robot speeds. Increase these numbers to trust the
   *                                 z axis speed state estimate less. In units of m/s.
   * @param accelMeasurementStdDevs  Standard deviations of the z axis robot acceleration measurements. Increase these
   *                                 numbers to trust the z axis robot acceleration measurements less. In units of m/s/s.
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
   *                                 with units in meters and radians.
   */
  public SwerveDrivePoseEstimator(
      Rotation3d gyroAngle,
      Pose3d initialPoseMeters,
      SwerveKinematics2 kinematics,
      SwerveModulePosition[] modulePositions,
      Matrix<N6, N1> stateStdDevs,
      Matrix<N6, N1> localMeasurementStdDevs,
      Matrix<N1, N1> accelStateStdDevs,
      Matrix<N1, N1> accelMeasurementStdDevs,
      Matrix<N6, N1> visionMeasurementStdDevs)
  {
    this(
        gyroAngle,
        initialPoseMeters,
        kinematics,
        modulePositions,
        stateStdDevs,
        localMeasurementStdDevs,
        accelStateStdDevs,
        accelMeasurementStdDevs,
        visionMeasurementStdDevs,
        0.02);
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to trust your model's
   *                                 state estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
   *                                 meters and radians.
   * @param localMeasurementStdDevs  Standard deviations of the encoder and gyro measurements. Increase these numbers to
   *                                 trust sensor readings from encoders and gyros less. This matrix is in the form
   *                                 [theta], with units in radians.
   * @param accelStateStdDevs        Standard deviations of the z axis robot speeds. Increase these numbers to trust the
   *                                 z axis speed state estimate less. In units of m/s.
   * @param accelMeasurementStdDevs  Standard deviations of the z axis robot acceleration measurements. Increase these
   *                                 numbers to trust the z axis robot acceleration measurements less. In units of m/s/s.
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
   *                                 with units in meters and radians.
   * @param nominalDtSeconds         The time in seconds between each robot loop.
   */
  @SuppressWarnings("ParameterName")
  public SwerveDrivePoseEstimator(
      Rotation3d gyroAngle,
      Pose3d initialPoseMeters,
      SwerveKinematics2 kinematics,
      SwerveModulePosition[] modulePositions,
      Matrix<N6, N1> stateStdDevs,
      Matrix<N6, N1> localMeasurementStdDevs,
      Matrix<N1, N1> accelStateStdDevs,
      Matrix<N1, N1> accelMeasurementStdDevs,
      Matrix<N6, N1> visionMeasurementStdDevs,
      double nominalDtSeconds)
  {
    m_nominalDt = nominalDtSeconds;

    m_previousModulePositions = modulePositions;

    m_observer =
        new UnscentedKalmanFilter<>(
            Nat.N6(),
            Nat.N6(),
            (x, u) -> u,
            (x, u) -> x,
            stateStdDevs,
            localMeasurementStdDevs,
            AngleStatistics.angleMean(2),
            AngleStatistics.angleMean(0),
            AngleStatistics.angleResidual(2),
            AngleStatistics.angleResidual(0),
            AngleStatistics.angleAdd(2),
            m_nominalDt);

    m_accelObserver =
    new UnscentedKalmanFilter<>(
        Nat.N1(),
        Nat.N1(),
        (x, u) -> u,
        (x, u) -> x.extractRowVector(0),
        accelStateStdDevs,
        accelMeasurementStdDevs,
        m_nominalDt);

    m_kinematics = kinematics;
    m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

    // Initialize vision R
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);

    m_visionCorrect =
        (u, y) ->
            m_observer.correct(
                Nat.N6(),
                u,
                y,
                (x, u1) -> x,
                m_visionContR,
                AngleStatistics.angleMean(2),
                AngleStatistics.angleResidual(2),
                AngleStatistics.angleResidual(2),
                AngleStatistics.angleAdd(2));

    m_gyroOffset = new Rotation3d();
    m_previousAngle = initialPoseMeters.getRotation();
    m_previousRoll = initialPoseMeters.getRotation().getX();
    m_previousPitch = initialPoseMeters.getRotation().getY();
    m_previousYaw = initialPoseMeters.getRotation().getZ();
    m_observer.setXhat(poseTo6dVector(initialPoseMeters));
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in vision measurements
   * after the autonomous period, or to change trust as distance to a vision target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
   *                                 with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N6, N1> visionMeasurementStdDevs)
  {
    m_visionContR = StateSpaceUtil.makeCovarianceMatrix(Nat.N6(), visionMeasurementStdDevs);
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>You NEED to reset your encoders (to zero) when calling this method.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param poseMeters The position on the field that your robot is at.
   * @param gyroAngle  The angle reported by the gyroscope.
   */
  public void resetPosition(Pose3d poseMeters, Rotation3d gyroAngle)
  {
    // Reset state estimate and error covariance
    m_observer.reset();
    m_latencyCompensator.reset();

    m_observer.setXhat(poseTo6dVector(poseMeters));

    m_gyroOffset = new Rotation3d();
    m_previousRoll = poseMeters.getRotation().getX();
    m_previousPitch = poseMeters.getRotation().getY();
    m_previousYaw = poseMeters.getRotation().getZ();
    m_previousAngle = poseMeters.getRotation();
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose3d getEstimatedPosition()
  {
    var a = new Pose3d(
      m_observer.getXhat(0), m_observer.getXhat(1), m_observer.getXhat(2), new Rotation3d(m_observer.getXhat(3), m_observer.getXhat(4), m_observer.getXhat(5)));
    // System.out.println(a);
    return a;
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose estimate while still
   * accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if you don't use your
   *                              own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime} then you
   *                              must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
   *                              timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
   *                              Timer.getFPGATimestamp as your time source or sync the epochs.
   */
  public void addVisionMeasurement(Pose3d visionRobotPoseMeters, double timestampSeconds)
  {
    m_latencyCompensator.applyPastGlobalMeasurement(
        Nat.N6(),
        m_observer,
        m_nominalDt,
        poseTo6dVector(visionRobotPoseMeters),
        m_visionCorrect,
        timestampSeconds);
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose estimate while still
   * accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to
   * {@link SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
   * @param timestampSeconds         The timestamp of the vision measurement in seconds. Note that if you don't use your
   *                                 own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime} then you
   *                                 must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
   *                                 timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should
   *                                 use Timer.getFPGATimestamp as your time source in this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
   *                                 with units in meters and radians.
   */
  public void addVisionMeasurement(
      Pose3d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N6, N1> visionMeasurementStdDevs)
  {
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be called every loop, and
   * the correct loop period must be passed into the constructor of this class.
   *
   * @param gyroAngle    The current gyro angle.
   * @param zAccel       The current acceleration in the z direction. In m/s/s.
   * @param moduleStates The current velocities and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose3d update(Rotation3d gyroAngle, Optional<Translation3d> accel, SwerveModuleState2[] moduleStates, SwerveModulePosition[] modulePositions)
  {
    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, accel, moduleStates, modulePositions);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be called every loop, and
   * the correct loop period must be passed into the constructor of this class.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle          The current gyroscope angle.
   * @param zAccel       The current acceleration in the z direction. In m/s/s.
   * @param moduleStates       The current velocities and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @SuppressWarnings("LocalVariableName")
  public Pose3d updateWithTime(
      double currentTimeSeconds, Rotation3d gyroAngle, Optional<Translation3d> accel, SwerveModuleState2[] moduleStates, SwerveModulePosition[] modulePositions)
  {
    double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);
    var omegaX = angle.minus(m_previousAngle).getX() / dt;
    var omegaY = angle.minus(m_previousAngle).getY() / dt;
    var omegaZ = angle.minus(m_previousAngle).getZ() / dt;

    var chassisSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
    var fieldRelativeVelocities =
        new Translation3d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0)
            .rotateBy(angle);
    var z = fieldRelativeVelocities.getZ();
    if (!accel.isEmpty())
    {
      var uZ = VecBuilder.fill(accel.get().getZ());
      var lZ = VecBuilder.fill(fieldRelativeVelocities.getZ());
  
      m_accelObserver.predict(uZ, dt);
      m_accelObserver.correct(uZ, lZ);
      z = m_accelObserver.getXhat(0);
    }

    var u = VecBuilder.fill(fieldRelativeVelocities.getX(), fieldRelativeVelocities.getY(), z, omegaX, omegaY, omegaZ);
    m_previousAngle = angle;
    System.out.println(fieldRelativeVelocities.toString());
    System.out.println(modulePositions[0].toString());

    m_previousPitch = Math.toRadians(placeInAppropriate0To360Scope(Math.toDegrees(m_previousPitch), Math.toDegrees(angle.getX())));
    m_previousRoll = Math.toRadians(placeInAppropriate0To360Scope(Math.toDegrees(m_previousRoll), Math.toDegrees(angle.getY())));
    m_previousYaw = Math.toRadians(placeInAppropriate0To360Scope(Math.toDegrees(m_previousYaw), Math.toDegrees(angle.getZ())));

    var moduleDeltas = new SwerveModulePosition[modulePositions.length];
    for (int index = 0; index < modulePositions.length; index++) {
      var current = modulePositions[index];
      var previous = m_previousModulePositions[index];

      moduleDeltas[index] =
          new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
      previous.distanceMeters = current.distanceMeters;
    }

    var angle_difference = angle.minus(m_previousAngle).getQuaternion().toRotationVector();
    System.out.println(moduleDeltas[0].toString());

    var twist2d = m_kinematics.toTwist2d(moduleDeltas);
    var twist =
        new Twist3d(
            twist2d.dx,
            twist2d.dy,
            0,
            angle_difference.get(0, 0),
            angle_difference.get(1, 0),
            angle_difference.get(2, 0));

    var newPose = getEstimatedPosition().exp(twist);
    // System.out.println(newPose.toString());

    var localY = VecBuilder.fill(
      newPose.getX(),
      newPose.getY(),
      newPose.getZ(),
      m_previousPitch, 
      m_previousRoll, 
      m_previousYaw);

    // System.out.println(localY);

    m_latencyCompensator.addObserverState(m_observer, u, localY, currentTimeSeconds);
    m_observer.predict(u, dt);
    m_observer.correct(u, localY);

    return getEstimatedPosition();
  }

  /**
   * Convert a {@link Pose3d} to a vector of [x, y, z, pitch, roll, yaw], where pitch, roll, yaw are in radians.
   *
   * @param pose A pose to convert to a vector.
   * @return The given pose in vector form.
   */
  public static Matrix<N6, N1> poseTo6dVector(Pose3d pose) {
    return VecBuilder.fill(
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getTranslation().getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  /**
   * Put an angle within the the 360 deg scope of a reference. For example, given a scope reference
   * of 756 degrees, assumes the full scope is (720-1080), and places an angle of 22 degrees into
   * it, returning 742 deg.
   *
   * @param scopeReference Current Angle (deg)
   * @param newAngle Target Angle (deg)
   * @return Closest angle within scope (deg)
   */
  private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = (scopeReference % 360);

    // Create the interval from the reference angle.
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    // Put the angle in the interval.
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    // Smooth the transition between interval boundaries.
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}