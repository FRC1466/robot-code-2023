// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ConversionConstants {
    public static final double
      GEAR_RATIO = 8.14 * 1.00415081 * Math.PI/2,
      CTRE_TICKS =  2048,
      CTRE_TICKS_PER_REV = CTRE_TICKS * GEAR_RATIO, // 26295
      WHEEL_DIAMETER = 1.975*2, //inches
      CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI,
      INCHES_PER_TICK = CIRCUMFERENCE / CTRE_TICKS_PER_REV,
      IPDS_TO_MPH = 0.568,
      IPDS_TO_MEPS = 0.254,
      METERS_PER_TICK = INCHES_PER_TICK / 37.3701,
      CTRE_NATIVE_TO_MPH = INCHES_PER_TICK * IPDS_TO_MPH,
      CTRE_NATIVE_TO_MPS = INCHES_PER_TICK * IPDS_TO_MEPS;

    public static double
      CHANGED_CTRE_TICKS_PER_REV = CTRE_TICKS_PER_REV;

  }

  public static final class DriveConstants {
    public static final int 
      FRONTRIGHT_PORT_DRIVE = 3,
      FRONTRIGHT_PORT_ROTATE = 4,
      FRONTRIGHT_PORT_CANCODER = 10,

      FRONTLEFT_PORT_DRIVE = 1,
      FRONTLEFT_PORT_ROTATE = 2,
      FRONTLEFT_PORT_CANCODER = 9,

      BACKRIGHT_PORT_DRIVE = 5,
      BACKRIGHT_PORT_ROTATE = 6,
      BACKRIGHT_PORT_CANCODER = 11,

      BACKLEFT_PORT_DRIVE = 7,
      BACKLEFT_PORT_ROTATE = 8,
      BACKLEFT_PORT_CANCODER = 12;

    public static double
      FRONTRIGHT_OFFSET = 135,
      FRONTLEFT_OFFSET = 45,
      BACKRIGHT_OFFSET = -10,
      BACKLEFT_OFFSET = -45;

    public static boolean
      FRONTRIGHT_DRIVEINVERT = false,
      FRONTLEFT_DRIVEINVERT = true,
      BACKRIGHT_DRIVEINVERT = true,
      BACKLEFT_DRIVEINVERT = false,
      
      FRONTRIGHT_ROTINVERT = false,
      FRONTLEFT_ROTINVERT = false,
      BACKRIGHT_ROTINVERT = false,
      BACKLEFT_ROTINVERT = false;
    
    public static final int
      GYRO_PORT = 20;

    public static final double 
      TRACKWIDTH_METERS = 0.375;

    public final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2), //frontleft
      new Translation2d(DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2), //frontright
      new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, DriveConstants.TRACKWIDTH_METERS/2), //backleft
      new Translation2d(-DriveConstants.TRACKWIDTH_METERS/2, -DriveConstants.TRACKWIDTH_METERS/2)); //backright
    // Drive limiters

    public static double 
      LIMIT_VX = 1.5,
      LIMIT_VY = 1.5,
      LIMIT_ROT = 4.0;


  }

  public static final class OIConstants {
    public static final int 
      DRIVER_PORT = 0,
      INTAKE_PORT = 1;
  }

  public static final class AutoConstants {
    public static final double
      MAX_SPEED_MPS = 3.0,
      MAX_ACCELERATION_MPS = 2.0;

    public static final Gains
      THETA_CONTROLLER = new Gains(1, 0, 0, 0, 0, 1),
      TRANSLATION_CONTROLLER = new Gains(1, 0, 0, 0, 0, 1);

      public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_SPEED_MPS, MAX_ACCELERATION_MPS);

  }

  public static final class IntakeConstants {

  }

  public static final class PIDConstants {
    public static final int 
      SLOT_IDX = 0,
      PID_LOOP_IDX = 0,
      TIMEOUT_MS = 30;

    public static final Gains 
      DRIVE_GAINS_VELOCITY  = new Gains(0.198, 0.00085, 4.0, 0,  0,  0.8),
      DRIVE_GAINS_POSITION  = new Gains(0.05, 0.00001, 0, 0,  0,  0.6);
  }

  public static final class DebugConstants {
    public static boolean
      isUsingWPIPID = false;
  }

}
