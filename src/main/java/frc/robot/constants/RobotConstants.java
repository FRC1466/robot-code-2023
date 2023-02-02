// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Gains;
import frc.lib.util.swerve.BetterSwerveKinematics;
import frc.lib.util.swerve.SwerveModuleConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants {

  public static final class Swerve {
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1, angleMotorID = 2, cancoderID = 9;
      public static final boolean driveInvert = false, angleInvert = true;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(-18.27 - 5.1 + 90 - 28.485);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID, angleMotorID, cancoderID, angleOffset, angleInvert, driveInvert);
    }

    /* Module Specific Constants */
    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3, angleMotorID = 4, cancoderID = 10;
      public static final boolean driveInvert = false, angleInvert = true;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.34 + 90 - 180 - 21.885);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID, angleMotorID, cancoderID, angleOffset, angleInvert, driveInvert);
    }

    /* Module Specific Constants */
    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5, angleMotorID = 6, cancoderID = 11;
      public static final boolean driveInvert = false, angleInvert = true;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(-53.87 + 90 - (11.05 / 2) - 10.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID, angleMotorID, cancoderID, angleOffset, angleInvert, driveInvert);
    }

    /* Module Specific Constants */
    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7, angleMotorID = 8, cancoderID = 12;
      public static final boolean driveInvert = false, angleInvert = true;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(7.56 - 12.75 + 90 - 16 - 9.044 + 8.345);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID, angleMotorID, cancoderID, angleOffset, angleInvert, driveInvert);
    }

    public static final double trackWidth = 0.4953,
        wheelDiameter = 0.10030,
        wheelCircumference = wheelDiameter * Math.PI;

    public static final BetterSwerveKinematics BETTER_KINEMATICS =
        new BetterSwerveKinematics(
            new Translation2d(-trackWidth / 2, trackWidth / 2), // frontleft
            new Translation2d(-trackWidth / 2, -trackWidth / 2), // frontright
            new Translation2d(trackWidth / 2, trackWidth / 2), // backleft
            new Translation2d(trackWidth / 2, -trackWidth / 2)); // backright

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(-trackWidth / 2, trackWidth / 2), // frontleft
            new Translation2d(-trackWidth / 2, -trackWidth / 2), // frontright
            new Translation2d(trackWidth / 2, trackWidth / 2), // backleft
            new Translation2d(trackWidth / 2, -trackWidth / 2)); // backright

    public static final double
        driveGearRatio = 8.2138672, // refine this for pose estimation (-16822)
        angleGearRatio = 12.839355527714421;

    public static final int gyroID = 20;

    public static final Gains driveGainsVelocity = new Gains(0.15, 0, 2.0, 0, 0, 0.8),
        driveGainsPosition = new Gains(0.050953, 0, 0.0014019, 0, 0, 0.6),
        headingGains = new Gains(0.118, 0.03, 0, 0, 0, 1.0);
    public static final int slotIdx = 0, pidLoopIdx = 0, timeoutMS = 30;
    public static final double MODULE_STEER_FF_CL = -0.30;
  }

  public static final class OIConstants {
    public static final int driverID = 4, intakeID = 1;

    public static final class InputLimits {
      public static double vx = -3.5,
          vxDeadband = 0.02,
          vy = 3.5,
          vyDeadband = 0.02,
          rad = -4.0,
          radDeadband = 0.10,
          slew = 0.5,
          reduced = 0.3,
          debounce = 0.1; // time in s
    }
  }

  public static final class AutoConstants {
    public static final double maxSpeedMPS = 2.0, maxAccelerationMPS = 1.0;

    public static final double balanceScale = 4.0;

    public static final Gains thetaController = new Gains(-20, 0.0, 0.0, 0, 0, 1),
        translationController = new Gains(8, 1.0, 0, 0, 0, 1);

    public static final List<AprilTag> aprilList =
        new ArrayList<>() {
          {
            add(
                new AprilTag(
                    1,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))));
            add(
                new AprilTag(
                    2,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))));
            add(
                new AprilTag(
                    3,
                    new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(174.19), // FIRST's
                        // diagram
                        // has a
                        // typo (it
                        // says
                        // 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI))));
            add(
                new AprilTag(
                    4,
                    new Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d(0.0, 0.0, Math.PI))));
            add(
                new AprilTag(
                    5,
                    new Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d())));
            add(
                new AprilTag(
                    6,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(174.19), // FIRST's
                        // diagram
                        // has a
                        // typo (it
                        // says
                        // 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d())));
            add(
                new AprilTag(
                    7,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d())));
            add(
                new AprilTag(
                    8,
                    new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d())));
          }
        };
    public static final double fieldLength = 16.54175;
    public static final double fieldWidth = 8.0137;
    public static final AprilTagFieldLayout apriltagFieldLayout =
        new AprilTagFieldLayout(aprilList, fieldLength, fieldWidth);
    public static final Translation3d cameraTranslation = new Translation3d(0.5, 0.0, 0.5);
    public static final Rotation3d cameraRotation = new Rotation3d();
  }

  public static final class IntakeConstants {}
}
