package frc.lib.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final boolean angleInvert;
  public final boolean driveInvert;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   * @param angleInvert
   * @param driveInvert
   */
  public SwerveModuleConstants(
      int driveMotorID,
      int angleMotorID,
      int canCoderID,
      Rotation2d angleOffset,
      boolean angleInvert,
      boolean driveInvert) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.driveInvert = driveInvert;
    this.angleInvert = angleInvert;
  }
}
