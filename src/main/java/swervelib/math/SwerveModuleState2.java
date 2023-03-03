package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Second order kinematics swerve module state. */
public class SwerveModuleState2 extends SwerveModuleState {

  /** Swerve module speed in meters per second. */
  public double speedMetersPerSecond = 0;

  public double accelMetersPerSecondSq = 0;
  /** Rad per sec */
  public double omegaRadPerSecond = 0;
  /** Swerve module angle as a {@link Rotation2d}. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleState2() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param accelMetersPerSecondSq The accel of the wheel of the module.
   * @param angle The angle of the module.
   * @param omegaRadPerSecond The angular velocity of the module.
   */
  public SwerveModuleState2(
      double speedMetersPerSecond,
      double accelMetersPerSecondSq,
      Rotation2d angle,
      double omegaRadPerSecond) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.accelMetersPerSecondSq = accelMetersPerSecondSq;
    this.angle = angle;
    this.omegaRadPerSecond = omegaRadPerSecond;
  }
}
