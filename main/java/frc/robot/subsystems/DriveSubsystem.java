// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX gyro_motor = new WPI_TalonSRX(DriveConstants.GYRO_PORT);
  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(gyro_motor);
  private final SwerveModule frontLeftModule = new SwerveModule(
    DriveConstants.FRONTLEFT_PORT_DRIVE, 
    DriveConstants.FRONTLEFT_PORT_ROTATE,
    DriveConstants.FRONTLEFT_PORT_CANCODER,
    DriveConstants.FRONTLEFT_OFFSET,
    false,
    DriveConstants.FRONTLEFT_DRIVEINVERT,
    DriveConstants.FRONTLEFT_ROTINVERT);
  private final SwerveModule frontRightModule = new SwerveModule(
    DriveConstants.FRONTRIGHT_PORT_DRIVE, 
    DriveConstants.FRONTRIGHT_PORT_ROTATE,
    DriveConstants.FRONTRIGHT_PORT_CANCODER,
    DriveConstants.FRONTRIGHT_OFFSET,
    false,
    DriveConstants.FRONTRIGHT_DRIVEINVERT,
    DriveConstants.FRONTRIGHT_ROTINVERT);
  private final SwerveModule backLeftModule = new SwerveModule(
    DriveConstants.BACKLEFT_PORT_DRIVE, 
    DriveConstants.BACKLEFT_PORT_ROTATE,
    DriveConstants.BACKLEFT_PORT_CANCODER,
    DriveConstants.BACKLEFT_OFFSET,
    false,
    DriveConstants.BACKLEFT_DRIVEINVERT,
    DriveConstants.BACKLEFT_ROTINVERT);
  private final SwerveModule backRightModule = new SwerveModule(
    DriveConstants.BACKRIGHT_PORT_DRIVE, 
    DriveConstants.BACKRIGHT_PORT_ROTATE,
    DriveConstants.BACKRIGHT_PORT_CANCODER,
    DriveConstants.BACKRIGHT_OFFSET,
    false,
    DriveConstants.BACKRIGHT_DRIVEINVERT,
    DriveConstants.BACKRIGHT_ROTINVERT);


  private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
  private SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);

  public SwerveModulePosition[] getCalculatedSwervePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftModule.getDrivePosition()*Constants.ConversionConstants.METERS_PER_TICK, moduleStates[0].angle),
      new SwerveModulePosition(frontRightModule.getDrivePosition()*Constants.ConversionConstants.METERS_PER_TICK, moduleStates[1].angle),
      new SwerveModulePosition(backLeftModule.getDrivePosition()*Constants.ConversionConstants.METERS_PER_TICK, moduleStates[2].angle),
      new SwerveModulePosition(backRightModule.getDrivePosition()*Constants.ConversionConstants.METERS_PER_TICK, moduleStates[3].angle)
    };
  }

  private final Pose2d initialPose = new Pose2d(5.0, 13.5, new Rotation2d(0));
  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, getGyroHeading(), getCalculatedSwervePositions(), initialPose);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.reset();

    SmartDashboard.putData("reset front left pos", new InstantCommand(() -> frontLeftModule.resetAngleEncoder(0)));
    SmartDashboard.putData("reset front right pos", new InstantCommand(() -> frontRightModule.resetAngleEncoder(0)));
    SmartDashboard.putData("reset back left pos", new InstantCommand(() -> backLeftModule.resetAngleEncoder(0)));
    SmartDashboard.putData("reset back right pos", new InstantCommand(() -> backRightModule.resetAngleEncoder(0)));
    SmartDashboard.putData("reset gyro position", new InstantCommand(() -> gyro.reset()));
  }
  
  /**
   * 
   * @return Rotation2d of gyro
   */
  public Rotation2d getGyroHeading() {
    SmartDashboard.putNumber("gyro rotation degrees", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("gyro pitch (degrees?)", gyro.getPitch());
    return gyro.getRotation2d();
  }

  /**
   * 
   * @return Pose2d of the robot from odometry
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * reset odometry of the robot from a given pose
   * @param pose Pose2d that the robot is at
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getGyroHeading(), getCalculatedSwervePositions(), pose);
  }

  /**
   * drive from module states list
   * @param states list of SwerveModuleStates that correspond to the robot
   */
  public void driveFromModuleStates(SwerveModuleState[] states) {
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
  }

  /**
   * update the odometry of the robot with current pose of the robot
   */
  public void updateRobotPose() {
    m_odometry.update(
      getGyroHeading(),
      getCalculatedSwervePositions()
      );
    SmartDashboard.putNumber("odometry x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("odometry rad", m_odometry.getPoseMeters().getRotation().getRadians());
  }

  public void resetAngleByCancoderOffset(double[] list) {
    frontLeftModule.resetAngleEncoder(list[0]);
    frontRightModule.resetAngleEncoder(list[1]);
    backLeftModule.resetAngleEncoder(list[2]);
    backRightModule.resetAngleEncoder(list[3]);
  }

  /**
   * drive robot from current module states in the class
   */
  public void drive() {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public void driveAlternate() {
    frontLeftModule.setDesiredStateCancoder(moduleStates[0]);
    frontRightModule.setDesiredStateCancoder(moduleStates[1]);
    backLeftModule.setDesiredStateCancoder(moduleStates[2]);
    backRightModule.setDesiredStateCancoder(moduleStates[3]);
  }

  /**
   * update speeds kinematics class 
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void updateSpeeds(double rad, double vx, double vy) {
    speeds.omegaRadiansPerSecond = rad;
    SmartDashboard.putNumber("speedsRad", rad);
    speeds.vxMetersPerSecond = vx;
    SmartDashboard.putNumber("speedsvx", vx);
    speeds.vyMetersPerSecond = vy;
    SmartDashboard.putNumber("speedsvy", vy);
  }

  /**
   * update speeds from field relative setup
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void updateSpeedsFieldRelative(double rad, double vx, double vy) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rad, getGyroHeading());
  }

  /**
   * update normal moduleStates
   */
  public void updateModuleStates() {
    moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
  }

  /**
   * update each module's inversion
   */
  public void updateModuleInversion() {
    frontLeftModule.changeMotorInversion(DriveConstants.FRONTLEFT_DRIVEINVERT, DriveConstants.FRONTLEFT_ROTINVERT);
    frontRightModule.changeMotorInversion(DriveConstants.FRONTRIGHT_DRIVEINVERT, DriveConstants.FRONTRIGHT_ROTINVERT);
    backLeftModule.changeMotorInversion(DriveConstants.BACKLEFT_DRIVEINVERT, DriveConstants.BACKLEFT_ROTINVERT);
    backRightModule.changeMotorInversion(DriveConstants.BACKRIGHT_DRIVEINVERT, DriveConstants.BACKRIGHT_ROTINVERT);
  }

  /**
   * update PID in the module substates from constants
   */
  public void updatePIDConfigs() {
    frontLeftModule.updatePID();
    frontRightModule.updatePID();
    backLeftModule.updatePID();
    backRightModule.updatePID();
  }

  public void setModulePositionPID(double p, double i, double d) {
    frontLeftModule.setRotationPID(p, i, d);
    frontRightModule.setRotationPID(p, i, d);
    backLeftModule.setRotationPID(p, i, d);
    backRightModule.setRotationPID(p, i, d);
  }

  /**
   * @return length of module states list
   */
  public int getStatesLength() {
    return moduleStates.length;
  }

  /**
   * get all motor errors
   * @return list of all errorstates of the motors
   */
  public double[][] getErrorStates() {
    return new double[][] {
      frontLeftModule.getErrorStates(),
      frontRightModule.getErrorStates(),
      backLeftModule.getErrorStates(),
      backRightModule.getErrorStates()
    };
  }

  /**
   * get all motor positions
   * @return list of all positions of the motors
   */
  public double[] getDrivePositions() {
    return new double[] {
      frontLeftModule.getDrivePosition(),
      frontRightModule.getDrivePosition(),
      backLeftModule.getDrivePosition(),
      backRightModule.getDrivePosition()
    };
  }

  /**
   * get all motor velocities
   * @return list of all velocities of the motors
   */
  public double[][] getVelocities() {

    return new double[][] {
      frontLeftModule.getVelocity(),
      frontRightModule.getVelocity(),
      backLeftModule.getVelocity(),
      backRightModule.getVelocity()
    };
  }

  public SwerveModuleState[] getCalculatedSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
  }


  @Override
  public void periodic() {
    updateRobotPose();
  }

}