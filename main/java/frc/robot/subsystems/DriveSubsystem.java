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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    initializeTelemetry();
    gyro.reset();
  }

  private GenericEntry gyroRotEntry;
  private GenericEntry gyroPitchEntry;
  private GenericEntry odometryXEntry;
  private GenericEntry odometryYEntry;
  private GenericEntry odometryDegEntry;
  /**
   * initializes telemetry
   */
  private void initializeTelemetry() {
    ShuffleboardTab teleTab = Shuffleboard.getTab("Telemetry");
    ShuffleboardLayout gyroLayout = teleTab
      .getLayout("gyro", BuiltInLayouts.kList)
      .withSize(1, 3);
    gyroLayout.add("reset gyro position", new InstantCommand(() -> gyro.reset()));
    gyroRotEntry = gyroLayout.add("gyro rotation deg", gyro.getRotation2d().getDegrees()).getEntry();
    gyroPitchEntry = gyroLayout.add("gyro pitch deg", gyro.getPitch()).getEntry();

    ShuffleboardLayout odometryLayout = teleTab
      .getLayout("odometry", BuiltInLayouts.kList)
      .withSize(1, 3);
      odometryXEntry = odometryLayout.add("x", m_odometry.getPoseMeters().getX()).getEntry();
      odometryYEntry = odometryLayout.add("y", m_odometry.getPoseMeters().getY()).getEntry();
      odometryDegEntry = odometryLayout.add("deg", m_odometry.getPoseMeters().getRotation().getDegrees()).getEntry();
  }
  
  /**
   * 
   * @return Rotation2d of gyro
   */
  public Rotation2d getGyroHeading() {
    gyroRotEntry.setDouble(gyro.getRotation2d().getDegrees());
    gyroPitchEntry.setDouble(gyro.getPitch());
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

  public void resetGyro() {
    gyro.reset();
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
    odometryXEntry.setDouble(m_odometry.getPoseMeters().getX());
    odometryYEntry.setDouble(m_odometry.getPoseMeters().getY());
    odometryDegEntry.setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());
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

  public void drivePosSpecificModule(double i) {
    frontLeftModule.setDrivePosition(i);
  }

  public void driveFromStopped() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI)));
    backRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI)));
  }

  /**
   * update speeds kinematics class 
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void updateSpeeds(double rad, double vx, double vy) {
    speeds.omegaRadiansPerSecond = rad;
    speeds.vxMetersPerSecond = vx;
    speeds.vyMetersPerSecond = vy;
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