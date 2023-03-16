package swervelib;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveModuleState2;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.simulation.SwerveModuleSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** The Swerve Module class which represents and controls Swerve Modules for the swerve drive. */
public class SwerveModule {

  /** Swerve module configuration options. */
  public final SwerveModuleConfiguration configuration;
  /** Angle offset from the absolute encoder. */
  private final double angleOffset;
  /** Swerve Motors. */
  private final SwerveMotor angleMotor, driveMotor;
  /** Absolute encoder for swerve drive. */
  private final SwerveAbsoluteEncoder absoluteEncoder;
  /**
   * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back
   * right.
   */
  public int moduleNumber;
  /** Feedforward for drive motor during closed loop control. */
  public SimpleMotorFeedforward feedforward;
  /** Timer to use for approximating module acceleration. */
  private final Timer timer;
  /** Last angle set for the swerve module. */
  public Rotation2d lastAngle;
  /** Last measured velocity the swerve module. */
  private double lastVelocity = 0;
  /** Last time on the timer. */
  private double lastTime;
  /** Simulated swerve module. */
  private SwerveModuleSimulation simModule;

  private boolean synchronizeEncoderQueued = false;

  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber Module number for kinematics.
   * @param moduleConfiguration Module constants containing CAN ID's and offsets.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration) {
    //    angle = 0;
    //    speed = 0;
    //    omega = 0;
    //    fakePos = 0;
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Initialize Feedforward for drive motor.
    feedforward = configuration.createDriveFeedforward();

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Configure voltage comp, current limit, and ramp rate.
    angleMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    driveMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    angleMotor.setCurrentLimit(configuration.physicalCharacteristics.angleMotorCurrentLimit);
    driveMotor.setCurrentLimit(configuration.physicalCharacteristics.driveMotorCurrentLimit);
    angleMotor.setLoopRampRate(configuration.physicalCharacteristics.angleMotorRampRate);
    driveMotor.setLoopRampRate(configuration.physicalCharacteristics.driveMotorRampRate);

    // Config angle encoders
    absoluteEncoder = moduleConfiguration.absoluteEncoder;
    if (absoluteEncoder != null) {
      absoluteEncoder.factoryDefault();
      absoluteEncoder.configure(moduleConfiguration.absoluteEncoderInverted);
      angleMotor.setPosition(getAbsolutePosition() - angleOffset);
    }

    // Config angle motor/controller
    angleMotor.configureIntegratedEncoder(moduleConfiguration.getPositionEncoderConversion(false));
    angleMotor.configurePIDF(moduleConfiguration.anglePIDF);
    angleMotor.configurePIDWrapping(-180, 180);
    angleMotor.setInverted(moduleConfiguration.angleMotorInverted);
    angleMotor.setMotorBrake(false);

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(moduleConfiguration.getPositionEncoderConversion(true));
    driveMotor.configurePIDF(moduleConfiguration.velocityPIDF);
    driveMotor.setInverted(moduleConfiguration.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    if (SwerveDriveTelemetry.isSimulation) {
      simModule = new SwerveModuleSimulation();
    }

    lastAngle = getState().angle;
  }

  /** Synchronize the integrated angle encoder with the absolute encoder. */
  public void queueSynchronizeEncoders() {
    if (absoluteEncoder != null) {
      synchronizeEncoderQueued = true;
    }
  }

  /**
   * Set the desired state of the swerve module.
   *
   * @param desiredState Desired swerve module state.
   * @param isOpenLoop   Whether to use open loop (direct percent) or direct velocity control.
   * @param force        Disables optimizations that prevent movement in the angle motor and forces the desired state
   *                     onto the swerve module.
   */
  public void setDesiredState(SwerveModuleState2 desiredState, boolean isOpenLoop, boolean force)
  {
    SwerveModuleState simpleState =
        new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
    desiredState =
        new SwerveModuleState2(
            0,
            simpleState.speedMetersPerSecond,
            desiredState.accelMetersPerSecondSq,
            simpleState.angle,
            desiredState.omegaRadPerSecond);
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber(
          "Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Optimized " + moduleNumber + " Angle Setpoint: ", desiredState.angle.getDegrees());
      SmartDashboard.putNumber(
          "Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));
    }

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / configuration.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      double velocity = desiredState.speedMetersPerSecond;
      if (velocity != lastVelocity) {
        driveMotor.setReference(velocity, feedforward.calculate(velocity));
      }
      lastVelocity = velocity;
    }

    Rotation2d angle = desiredState.angle;

    // If we are forcing the angle

    // Prevents module rotation if speed is less than 1%
    if (!force) {
      angle = Math.abs(desiredState.speedMetersPerSecond) <= (configuration.maxSpeed * 0.01) ? lastAngle : angle;
    }
    // Prevent module rotation if angle is the same as the previous angle.
    if (angle != lastAngle)
    {
      if (absoluteEncoder != null && synchronizeEncoderQueued) {
        double absoluteEncoderPosition = getAbsolutePosition();
        angleMotor.setPosition(absoluteEncoderPosition - angleOffset);
        angleMotor.setReference(
                angle.getDegrees(),
                Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKV,
                absoluteEncoderPosition);
        synchronizeEncoderQueued = false;
      } else {
        angleMotor.setReference(
                angle.getDegrees(),
                Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKV);
      }
    }
    lastAngle = angle;

    if (SwerveDriveTelemetry.isSimulation) {
      simModule.updateStateAndPosition(desiredState);
    }
  }

  /**
   * Set the angle for the module.
   *
   * @param angle Angle in degrees.
   */
  public void setAngle(double angle) {
    angleMotor.setReference(angle, configuration.angleKV);
    lastAngle = Rotation2d.fromDegrees(angle);
  }

  /**
   * Get the Swerve Module state.
   *
   * @return Current SwerveModule state.
   */
  public SwerveModuleState2 getState() {
    double position;
    double velocity;
    double accel;
    Rotation2d azimuth;
    double omega;
    var dt = timer.get() - lastTime;
    lastTime = timer.get();
    if (!SwerveDriveTelemetry.isSimulation) {
      position = driveMotor.getPosition();
      velocity = driveMotor.getVelocity();
      accel = (velocity - lastVelocity) / dt;
      lastVelocity = velocity;
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
      omega = Math.toRadians(angleMotor.getVelocity());
    } else {
      return simModule.getState();
    }
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
    }
    return new SwerveModuleState2(position, velocity, accel, azimuth, omega);
  }

  /**
   * Get the absolute position. Falls back to relative position on reading failure.
   *
   * @return Absolute encoder angle in degrees.
   */
  public double getAbsolutePosition() {
    if (absoluteEncoder != null) {
      double angle = absoluteEncoder.getAbsolutePosition();
      if (absoluteEncoder.readingError) {
        angle = getRelativePosition();
      }

      return angle;
    }

    return getRelativePosition();
  }

  /**
   * Get the relative angle in degrees.
   *
   * @return Angle in degrees.
   */
  public double getRelativePosition() {
    return angleMotor.getPosition();
  }

  /**
   * Set the brake mode.
   *
   * @param brake Set the brake mode.
   */
  public void setMotorBrake(boolean brake) {
    driveMotor.setMotorBrake(brake);
  }

  /** Reset the drive motor position to 0. */
  public void resetEncoder() {
    driveMotor.setPosition(0);
  }
}
