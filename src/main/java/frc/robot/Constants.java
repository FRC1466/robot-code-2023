// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class OIConstants {
    public static final int driverID = 4, intakeID = 5;

    public static final class InputLimits {
      public static double vxDeadband = 0.02, vyDeadband = 0.02, radDeadband = 0.10, reduced = 0.3;
    }
  }

  public static final class AutoConstants {
    public static final double maxSpeedMPS = 2.0, maxAccelerationMPS = 3.0;
    public static final double balanceScale = 1.0, balanceScalePow = 1.0;
    public static final Gains thetaController = new Gains(5, 0.0, 0.0, 0, 0, 1),
        translationController = new Gains(1, 0, 0, 0, 0, 1);

    public static final LoadingArea loadingArea =
        new LoadingArea(
            new RectanglePoseArea(new Translation2d(9.91, 6.82), new Translation2d(16.24, 7.97)),
            new RectanglePoseArea(new Translation2d(13.24, 5.66), new Translation2d(16.51, 7.97)),
            new HolonomicPose2d(new Pose2d(15.79, 7.34, new Rotation2d()), new Rotation2d()),
            new HolonomicPose2d(new Pose2d(15.75, 6.00, new Rotation2d()), new Rotation2d()));

    public static final List<ScoringArea> scoreAreaList =
        new ArrayList<>() {
          {
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)),
                    // diagonal y's should not overlap
                    new HolonomicPose2d(new Pose2d(1.62, 4.95, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 4.40, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 3.84, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
                    new HolonomicPose2d(new Pose2d(1.62, 3.30, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 2.72, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 2.19, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
                    new HolonomicPose2d(new Pose2d(1.62, 1.61, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 1.03, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 0.55, new Rotation2d()), new Rotation2d())));
          }
        };
    public static final Translation3d cameraTranslation = new Translation3d(0.5, 0.0, 0.5);
    public static final Rotation3d cameraRotation = new Rotation3d(0, 0, 0);
  }

  public static final class ArmConstants {
    public static final int armPort = 30;
    public static final Gains armPosition = new Gains(2.8, 0, 0.5, 0, 0, 0.6);

    public static final class ArmConfig {
      public TalonFXConfiguration config;

      public ArmConfig() {
        config = new TalonFXConfiguration();
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = ArmConstants.armPosition.peakOutput;
        config.peakOutputReverse = -ArmConstants.armPosition.peakOutput;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
      }
    }
  }

  public static final class GripperConstants {
    public static final Gains gripperPosition = new Gains(0, 0, 0, 0, 0, 0.5);
  }

  public static final class LEDConstants {
    public static final int PWMPort = 9, length = 10;
  }
}
