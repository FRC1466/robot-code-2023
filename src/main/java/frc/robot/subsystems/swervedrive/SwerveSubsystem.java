// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants.InputLimits;
import java.io.File;
import java.util.List;
import java.util.function.Supplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.Matter;
import swervelib.math.SwerveKinematics2;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import webblib.util.swerve.SwerveBalance;

public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  private final SwerveBalance swerveBalance;

  public boolean softVisionMeasurements = true;

  private final PhotonCameraWrapper photon;

  private final Supplier<Translation3d> armCOM;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory, Supplier<Translation3d> armCOM) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveBalance = new SwerveBalance(Auton.balanceScale, Auton.balanceScalePow);
    swerveDrive.swerveController.addSlewRateLimiters(
        new SlewRateLimiter(InputLimits.vxSlew),
        new SlewRateLimiter(InputLimits.vySlew),
        new SlewRateLimiter(InputLimits.angSlew));
    photon = new PhotonCameraWrapper();
    this.armCOM = armCOM;
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop Whether to use closed-loop velocity control. Set to true to disable
   *     closed-loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop, false);
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    var pose = photon.getEstimatedGlobalPose(swerveDrive.getPose3d());
    // pose.ifPresent(
    //     estimatedRobotPose -> {
    //       var adjPose = new Pose3d(estimatedRobotPose.estimatedPose.getTranslation(),
    // getPose3d().getRotation());
    //       swerveDrive.addVisionMeasurement(
    //             adjPose,
    //             estimatedRobotPose.timestampSeconds,
    //             true,
    //             0.5);
    //     });
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveKinematics2} of the swerve drive.
   */
  public SwerveKinematics2 getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(new Pose3d(initialHolonomicPose));
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Pose3d getPose3d() {
    return swerveDrive.getPose3d();
  }

  /**
   * Set field-relative chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Field-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var translation = SwerveMath.limitVelocity(new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond), this.getFieldVelocity(), this.getPose(),
            Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS, new Matter(armCOM.get(), Constants.ARM_MASS)),
            this.getSwerveDriveConfiguration());
    SmartDashboard.putString("LimitedTranslation", translation.toString());
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Gets plane inclination with current robot plane and the plane z = 0.
   *
   * @return plane inclination in radians.
   */
  public Rotation2d getPlaneInclination() {
    return Rotation2d.fromRadians(
        Math.atan(Math.hypot(swerveDrive.getPitch().getTan(), swerveDrive.getRoll().getTan())));
  }

  /**
   * Gets the translation of the robot according to the swerve balance updater.
   *
   * @return translation of the robot.
   */
  public Translation2d getBalanceTranslation() {
    return swerveBalance.calculate(swerveDrive.getGyroRotation3d()).unaryMinus();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorIdleMode(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, headingX, headingY, getHeading().getRadians());
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, angle.getRadians(), getHeading().getRadians());
  }

  public boolean isMoving() {
    return Math.abs(getFieldVelocity().vxMetersPerSecond) > 0.1
        || Math.abs(getFieldVelocity().vxMetersPerSecond) > 0.1
        || Math.abs(getFieldVelocity().omegaRadiansPerSecond) > 0.1;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   * Forcing the robot to keep the current pose.
   */
  public void lockPose() {
    swerveDrive.lockPose();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }
}
