// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import webblib.util.Gains;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;
import webblib.util.chargedup.LoadingArea;
import webblib.util.chargedup.ScoringArea;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (1000) * 0.453592; // 32lbs * kg per pound
  public static final double ARM_MASS = 100.0;
  public static final Translation3d INITIAL_ARM_MOUNT = new Translation3d(0.3, 0, 0.7);
  public static final double ARM_LENGTH = 0.7;
  public static final double CHASSIS_MASS = ROBOT_MASS - ARM_MASS;
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(0.0)), ROBOT_MASS);
  ;
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag
  public static final double STOP_SECONDS = 3.0;

  public static final class OIConstants {
    public static final int driverID = 4, intakeID = 5, assistantID = 2;

    public static final class InputLimits {
      public static final double vxDeadband = 0.02,
          vyDeadband = 0.02,
          angDeadband = 0.10,
          reduced = 0.5;
      public static final double defaultAngScale = 0.8;
      public static final double vxSlew = 8.0, vySlew = 8.0, angSlew = 8.0;
    }
  }

  public static final class Auton {
    public static final PIDFConfig xAutoPID = new PIDFConfig(5.0, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(5.0, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(4.2, 0, 0.0);

    public static final double maxSpeedMPS = 2.5;
    public static final double maxAccelerationMPS = 1.8;
    public static final double balanceScale = 2.0, balanceScalePow = 1.0, balanceLimitDeg = 2.0;

    public static final LoadingArea loadingArea =
        new LoadingArea(
            new RectanglePoseArea(new Translation2d(9.91, 6.82), new Translation2d(16.24, 7.97)),
            new RectanglePoseArea(new Translation2d(13.24, 5.66), new Translation2d(16.51, 7.97)),
            new HolonomicPose2d(new Pose2d(15.79, 7.34, new Rotation2d()), new Rotation2d()),
            new HolonomicPose2d(new Pose2d(15.75, 6.00, new Rotation2d()), new Rotation2d()));

    public static final double lineUpMid = 1.73;
    public static final List<ScoringArea> scoreAreaList =
        new ArrayList<>() {
          {
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)),
                    // diagonal y's should not overlap
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 4.95, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 4.40, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 3.84, new Rotation2d(Math.PI)), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
                    new HolonomicPose2d(
                        new Pose2d(1.73, 3.30, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.73, 2.72, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.73, 2.19, new Rotation2d(Math.PI)), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 1.61, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 1.03, new Rotation2d(Math.PI)), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(lineUpMid, 0.55, new Rotation2d(Math.PI)), new Rotation2d())));
          }
        };
    public static final Translation3d cameraTranslation = new Translation3d(0.28, 0.0, 0.23);
    public static final Rotation3d cameraRotation = new Rotation3d(0, Math.toRadians(-15), 0);
  }

  public static final class ArmConstants {
    public static final int armPort = 30, dutyCyclePort = 0;
    public static final Gains armPosition = new Gains(0.95, 0, 0, 0, 0, 1.0);
    public static final double dutyCycleResolution = 1.0;
    public static final double absolutePositionOffset = 0.231143;
    public static final double maxRadians = 4.29, loftRadians = maxRadians - 0.3;
    public static final double minRadians = -0.75;
    public static final double launchRadians = -0.25;
    public static final double stationDegrees = 148.0,
        midDegrees = 149.0,
        midDegreesScore = 165.0,
        highDegrees = 135.0,
        highLaunchDegrees = 195.0,
        verticalDegrees = 90.0;
    public static final double toleranceRadians = 0.10;
    public static final double armInputScale = 2 * Math.PI / (maxRadians - minRadians);
    public static final double armOffset = minRadians + (maxRadians - minRadians) / 2;
    public static final double gravityFF = 0.05;
    public static final boolean encoderInverted = true;
    public static final double overrideFFScale = 0.15;

    public static final class ArmConfig {
      public static final SupplyCurrentLimitConfiguration supplyCurrent;
      public static final StatorCurrentLimitConfiguration statorCurrent;
      public static final TalonFXConfiguration motorConfig;

      static {
        supplyCurrent = new SupplyCurrentLimitConfiguration();
        supplyCurrent.enable = true;
        supplyCurrent.currentLimit = 40;
        statorCurrent = new StatorCurrentLimitConfiguration();
        statorCurrent.enable = true;
        statorCurrent.currentLimit = 40;

        motorConfig = new TalonFXConfiguration();
        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = ArmConstants.armPosition.peakOutput;
        motorConfig.peakOutputReverse = -ArmConstants.armPosition.peakOutput;
        motorConfig.supplyCurrLimit = supplyCurrent;
        motorConfig.statorCurrLimit = statorCurrent;
        motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
      }
    }
  }

  public static final class Intake {
    public static final double intakeV = 5.6,
        dropV = -0.85,
        launchV = -4.7,
        powerLaunchV = -5.2,
        holdV = 2.8;
    public static final double stallCurrent = 19.0;
    public static final double stallSeconds = 0.8;

    public static final int motorID = 34;
  }

  public static final class PoseEstimator {
    /** THANK YOU IRON PANTHERS */
    public static final double NOISY_DISTANCE_METERS = 2.5;

    /**
     * The number to multiply by the smallest of the distance minus the above constant, clamped
     * above 1 to be the numerator of the fraction.
     */
    public static final double DISTANCE_WEIGHT = 7;

    /**
     * The number to multiply by the number of tags beyond the first to get the denominator of the
     * deviations matrix.
     */
    public static final double TAG_PRESENCE_WEIGHT = 10;

    /** The amount to shift the pose ambiguity by before multiplying it. */
    public static final double POSE_AMBIGUITY_SHIFTER = .2;

    /** The amount to multiply the pose ambiguity by if there is only one tag. */
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  }

  public static final class LEDConstants {
    public static final int PWMPort = 9;
  }

  public static final class PDHConstants {
    public static final int port = 50;
  }
}
