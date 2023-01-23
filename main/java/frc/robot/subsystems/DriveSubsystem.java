// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RectanglePoseArea;
import frc.lib.util.swerve.BetterSwerveDrivePoseEstimator;
import frc.lib.util.swerve.SwerveBalance;
import frc.robot.constants.RobotConstants.Swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_Pigeon2 gyro;
  private final SwerveModule[] swerveModules;
  private ChassisSpeeds speeds;
  private SwerveModuleState[] desiredModuleStates;
  private BetterSwerveDrivePoseEstimator swervePoseEstimator;
  private PIDController headingController = new PIDController(0.8, 0, 0);
  private SwerveBalance swerveBalance = new SwerveBalance();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro = new WPI_Pigeon2(Swerve.gyroID);
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Swerve.Mod0.constants),
      new SwerveModule(1, Swerve.Mod1.constants),
      new SwerveModule(2, Swerve.Mod2.constants),
      new SwerveModule(3, Swerve.Mod3.constants),
    };

    initializeTelemetry();

    swervePoseEstimator = new BetterSwerveDrivePoseEstimator(Swerve.KINEMATICS, getGyroRotation2d(), getSwervePositions(), new Pose2d());
    speeds = new ChassisSpeeds();
    desiredModuleStates = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
    
    gyro.reset();
  }

  private GenericEntry gyroRotEntry;
  private GenericEntry gyroPitchEntry;
  private GenericEntry gyroRollEntry;
  private GenericEntry odometryXEntry;
  private GenericEntry odometryYEntry;
  private GenericEntry odometryDegEntry;
  private GenericEntry[] cancoderEntries;
  private GenericEntry[] integratedEntries;
  private GenericEntry[] velEntries;
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
    gyroRollEntry = gyroLayout.add("gyro roll deg", gyro.getRoll()).getEntry();

    ShuffleboardLayout odometryLayout = teleTab
      .getLayout("odometry", BuiltInLayouts.kList)
      .withSize(1, 3);
    odometryXEntry = odometryLayout.add("x", 0.0).getEntry();
    odometryYEntry = odometryLayout.add("y", 0.0).getEntry();
    odometryDegEntry = odometryLayout.add("deg", 0.0).getEntry();

    cancoderEntries = new GenericEntry[4];
    integratedEntries = new GenericEntry[4];
    velEntries = new GenericEntry[4];
    for(SwerveModule module : swerveModules){
      ShuffleboardLayout encoderLayout = teleTab
        .getLayout("Module " + module.moduleNumber + " Encoders", BuiltInLayouts.kList)
        .withSize(1, 3);
      cancoderEntries[module.moduleNumber] = encoderLayout.add("Cancoder", module.getCancoderAngle().getDegrees()).getEntry();
      integratedEntries[module.moduleNumber] = encoderLayout.add("Integrated", module.getPosition().angle.getDegrees()).getEntry();
      velEntries[module.moduleNumber] = encoderLayout.add("Velocity", module.getState().speedMetersPerSecond).getEntry();    
    }
    
  }

  /**
   * updates telemetry of modules
   */
  public void updateModuleTelemetry() {
    for(var module : swerveModules){ 
      cancoderEntries[module.moduleNumber].setDouble(module.getCancoderAngle().getDegrees());
      SmartDashboard.putNumber("module " + module.moduleNumber + "cancoder deg", module.getCancoderAngle().getDegrees());
      integratedEntries[module.moduleNumber].setDouble(module.getPosition().angle.getDegrees());
      velEntries[module.moduleNumber].setDouble(module.getState().speedMetersPerSecond);
    }
  }
  
  /**
   * @return Rotation2d of gyro yaw
   */
  public Rotation2d getGyroRotation2d() {
    gyroRotEntry.setDouble(gyro.getRotation2d().getDegrees());
    return gyro.getRotation2d();
  }

  /**
   * @return Rotation2d of gyro pitch
   */
  public Rotation2d getGyroPitch() {
    gyroPitchEntry.setDouble(gyro.getPitch());
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  /**
   * @return Rotation2d of gyro pitch
   */
  public Rotation2d getGyroRoll() {
    gyroRollEntry.setDouble(gyro.getRoll());
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  /**
   * resets gyro
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * 
   * @return Pose2d of the robot from pose estimator
   */
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speeds;
  }

  public boolean isPoseWithinArea(RectanglePoseArea area) {
    return area.isPoseWithinArea(getPose());
  }

  /**
   * reset odometry of the robot from a given pose
   * @param pose Pose2d that the robot is at
   */
  public void resetPose(Pose2d pose) {
    swervePoseEstimator.resetPosition(getGyroRotation2d(), getSwervePositions(), pose);
  }

  /**
   * reset modules to their absolute position
   */
  public void resetModulesToAbsolute() {
    for(var modules : swerveModules){
      modules.resetToAbsolute();
  }
  }

  /**
   * drive from module states list
   * @param states list of SwerveModuleStates that correspond to the robot
   */
  public void setDesiredModuleStates(SwerveModuleState[] states) {
    for (var module : swerveModules) {
      module.setDesiredState(states[module.moduleNumber]);
    }
  }

  /**
   * update the odometry of the robot with current pose of the robot
   */
  public void updateRobotPose() {
    swervePoseEstimator.update(
      getGyroRotation2d(),
      getGyroPitch(),
      getGyroRoll(),
      getSwervePositions()
      );
    odometryXEntry.setDouble(swervePoseEstimator.getEstimatedPosition().getX());
    odometryYEntry.setDouble(swervePoseEstimator.getEstimatedPosition().getY());
    odometryDegEntry.setDouble(swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public void driveAutoBalancingFull() {
    speeds = swerveBalance.update(getGyroPitch(), getGyroRoll());
    updateModuleStates();
    drive();
  }

  /**
   * drive robot from current module states in the class
   */
  public void drive() {
    for (var module : swerveModules) {
      module.setDesiredState(desiredModuleStates[module.moduleNumber]);
    }
  }

  /**
   * drive a specific swerve module only by position (only front left)
   * @param i double from [-1, 1]
   */
  public void drivePosSpecificModule(double i) {
    swerveModules[0].setDrivePosition(i);
  }

  /**
   * set modude positions to a locked position with vel pid set to 0 to attempt to brake
   */
  public void driveFromStopped() {
    var stopped = new SwerveModuleState(0, new Rotation2d(Math.PI/2));
    for (var module : swerveModules) {
      module.setDesiredState(stopped);
    }
  }

  /**
   * update speeds kinematics class 
   * @param rad rad/s speed of robot
   * @param vx forward velocity in m/s
   * @param vy sideways velocity in m/s
   * @param isFieldRelative the state of field relative
   */
  public void setSpeeds(double rad, double vx, double vy, boolean isFieldRelative) {
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 
      rad, getGyroRotation2d());
    } else {
      speeds.omegaRadiansPerSecond = rad + headingController.calculate(Rotation2d.fromDegrees(gyro.getRate()).getRadians(), rad);
      speeds.vxMetersPerSecond = vx;
      speeds.vyMetersPerSecond = vy;
    } 
  }

    /**
   * update speeds from field relative setup
   * @param rad rad/s speed of robot
   * @param vx horizontal velocity in m/s
   * @param vy vertical velocity in m/s
   */
  public void setSpeedsFieldRelativeAlternate(double rad, double vx, double vy) {
    var robot_pose_vel = new Pose2d(vx * 0.02, vy * 0.02, Rotation2d.fromRadians(rad * 0.02));
    var twist_vel = getPose().log(robot_pose_vel);
    var adjSpeeds = new ChassisSpeeds(twist_vel.dx / 0.02, twist_vel.dy / 0.02, twist_vel.dtheta / 0.02);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(adjSpeeds.vxMetersPerSecond, adjSpeeds.vyMetersPerSecond, adjSpeeds.omegaRadiansPerSecond, getGyroRotation2d());
  }

  /**
   * update normal moduleStates
   */
  public void updateModuleStates() {
    desiredModuleStates = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
    // for (int i = 0; i < desiredModuleStates.length; i++) {
    //   desiredModuleStates[i].angle = Rotation2d.fromRadians(desiredBetterModuleState[i].angle.getRadians() + desiredBetterModuleState[i].omegaRadPerSecond * Swerve.MODULE_STEER_FF_CL * 0.065);
    //   desiredModuleStates[i].speedMetersPerSecond = desiredBetterModuleState[i].speedMetersPerSecond;
    // }
    SmartDashboard.putString("test3", desiredModuleStates.toString());
  }

  /**
   * update PID in the module substates from constants
   */
  public void updatePIDConfigs() {
    for (SwerveModule module : swerveModules) {
      module.updatePID();
    }
  }

  /**
   * @return SwerveModuleState[] of all modules, calculated from drive velocity and cancoders
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    var states = new SwerveModuleState[4];
    for(var module : swerveModules){
        states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * @return SwerveModulePosition[] of all modules, calculated from drive position and cancoders
   */
  public SwerveModulePosition[] getSwervePositions() {
    var positions = new SwerveModulePosition[4];
    for(var module : swerveModules){
        positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }


  @Override
  public void periodic() {
    updateRobotPose();
    updateModuleTelemetry();
  }

}