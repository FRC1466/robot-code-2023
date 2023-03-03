package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Second order kinematics swerve module state. */
public class SwerveModulePosition2 extends SwerveModuleState {

  /** Swerve module speed in meters per second. */
  public double distanceMeters = 0;
  public double velocityMetersPerSecond = 0;
  /** Rad per sec */
  public double omegaRadPerSecond = 0;
  /** Swerve module angle as a {@link Rotation2d}. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModulePosition2() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param distanceMeters The position of the wheel of the module.
   * @param velocityMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   * @param omegaRadPerSecond The angular velocity of the module.
   */
  public SwerveModulePosition2(
      double distanceMeters, double velocityMetersPerSecond, Rotation2d angle, double omegaRadPerSecond) {
    this.distanceMeters = distanceMeters;
    this.velocityMetersPerSecond = velocityMetersPerSecond;
    this.angle = angle;
    this.omegaRadPerSecond = omegaRadPerSecond;
  }
}
