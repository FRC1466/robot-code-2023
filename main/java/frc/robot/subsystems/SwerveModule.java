package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.PIDConstants;

public class SwerveModule {
    private WPI_TalonFX[] motors;
    private WPI_CANCoder cancoder;
    private double m_cancoderOffset;
    private boolean m_RotInverted;
    private boolean m_driveInverted;
    private PIDController rotPID = new PIDController(
        PIDConstants.DRIVE_GAINS_POSITION.P,
        PIDConstants.DRIVE_GAINS_POSITION.I,
        PIDConstants.DRIVE_GAINS_POSITION.D);
    
    
    /**
     * Initialize a Swerve Module
     */
    public SwerveModule(
        int drivePort,
        int rotationPort,
        int cancoderPort,
        double cancoderOffset,
        boolean isRotationOffset,
        boolean driveInverted,
        boolean rotInverted
    ) {
        motors = new WPI_TalonFX[] {
            new WPI_TalonFX(drivePort),
            new WPI_TalonFX(rotationPort)
        }; 

        cancoder = new WPI_CANCoder(cancoderPort);

        m_RotInverted = rotInverted;
        m_driveInverted = driveInverted;
        m_cancoderOffset = cancoderOffset;

        rotPID.enableContinuousInput(-ConversionConstants.CTRE_TICKS_PER_REV/2, ConversionConstants.CTRE_TICKS_PER_REV/2);


        initializeMotors();
        initializeMotorsPID();
        initializeCancoder();
        // resetAngleEncoder((cancoder.getAbsolutePosition()+cancoderOffset)/360 * ConversionConstants.CTRE_TICKS_PER_REV);
        resetDriveEncoder(0.0);
        // cancoder.setPosition(cancoder.getAbsolutePosition()+cancoderOffset);
    }

    /**
     * get current state of swerve module
     * @return SwerveModuleState created from current encoder layouts
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            motors[0].getSelectedSensorVelocity() * ConversionConstants.CTRE_NATIVE_TO_MPS, 
            new Rotation2d(motors[1].getSelectedSensorPosition() * ConversionConstants.CTRE_TICKS_PER_REV % ConversionConstants.CTRE_TICKS_PER_REV)
            );
    }

    /**
     * get list of errors
     * @return list of doubles for both motor errors, 0: drive, 1: position
     */
    public double[] getErrorStates() {
        return new double[] {motors[0].getClosedLoopError(), motors[1].getClosedLoopError()};
    }

    /**
     * 
     * @return drive position in raw ctre units
     */
    public double getDrivePosition() {
        return motors[0].getSelectedSensorPosition();
    }

    /**
     * 
     * @return position of rotation motor in raw ctre units
     */
    public double getRotationPosition() {
        return motors[1].getSelectedSensorPosition();
    }

    public Rotation2d getCancoderAngle() {
        return Rotation2d.fromDegrees(
            wrapCancoderOutput(
                cancoder.getAbsolutePosition() + m_cancoderOffset
            ));
    }

    /**
     * get list of velocities
     * @return list of doubles for both motor velocities, 0: drive, 1: position
     */
    public double[] getVelocity() {
        return new double[] {motors[0].getSelectedSensorVelocity(), motors[1].getSelectedSensorVelocity()};
    }

    /**
     * @param currentAngle        what the controller currently reads (radians)
     * @param targetAngleSetpoint the desired angle (radians)
     * @return the target angle in controller's scope (radians)
     */
    private double convertAngleToSetPoint(double currentAngle, double targetAngleSetpoint) {
        targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, Math.PI * 2); //this function has a very specific usecase lol

        double remainder = currentAngle % (Math.PI * 2);
        double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

        // We don't want to rotate over 180 degrees, so just rotate the other way (add a
        // full rotation)
        if (adjustedAngleSetpoint - currentAngle > Math.PI) {
            adjustedAngleSetpoint -= Math.PI * 2;
        } else if (adjustedAngleSetpoint - currentAngle < -Math.PI) {
            adjustedAngleSetpoint += Math.PI * 2;
        }

        return adjustedAngleSetpoint;

    }

     /**
     * @param position position of cancoder in degrees
     * @return         adjusted degree within a [-180, 180] wrapped output
     */
    private double wrapCancoderOutput(double position) {
        double m = Math.floor((Math.abs(position)-180) / 360);

        if (position > 180.0) {
            position =- 360.0 * (m + 1);
        }
        if (position < -180.0) {
            position += 360.0 * (m + 1);
        }

        return position;
    }

    /**
     * set motor velocity and position from a state using PID from Talons
     * @param desiredState SwerveModuleState, unoptimized, that corresponds to the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state =
            SwerveModuleState.optimize(
                desiredState, 
                new Rotation2d(0));
        
        double unitsVel = desiredState.speedMetersPerSecond / ConversionConstants.CTRE_NATIVE_TO_MPS;
        motors[0].set(TalonFXControlMode.Velocity, unitsVel);

        SmartDashboard.putNumber("ANGLESTATE", desiredState.angle.getRadians());

        double ticks = ConversionConstants.CHANGED_CTRE_TICKS_PER_REV;

        double setpoint = convertAngleToSetPoint(
            (getRotationPosition()/ticks) *2*Math.PI, 
            desiredState.angle.getRadians()) / (2*Math.PI) * ticks;
        // setpoint = desiredState.angle.getRadians() / (2*Math.PI) * ConversionConstants.CTRE_TICKS_PER_REV;

        SmartDashboard.putNumber("SETPOINT", setpoint);
        motors[1].set(TalonFXControlMode.Position, setpoint);
    }

    /**
     * set motor velocity and position from a state using inbuilt PID with wpilib (for position only)
     */
    public void setDesiredStatePID(SwerveModuleState desiredState) {
        double unitsVel = desiredState.speedMetersPerSecond / ConversionConstants.CTRE_NATIVE_TO_MPS;
        motors[0].set(TalonFXControlMode.Velocity, unitsVel);

        SmartDashboard.putNumber("ANGLESTATE", desiredState.angle.getRadians());

        double ticks = ConversionConstants.CHANGED_CTRE_TICKS_PER_REV;

        double setpoint =
            desiredState.angle.getRadians() / (2*Math.PI) * ticks;
        // setpoint = desiredState.angle.getRadians() / (2*Math.PI) * ConversionConstants.CTRE_TICKS_PER_REV;

        SmartDashboard.putNumber("SETPOINT", setpoint);
        double pidOutput = MathUtil.clamp(rotPID.calculate(getRotationPosition(), setpoint), -0.8, 0.8);
        SmartDashboard.putNumber("PID OUTPUT", pidOutput);
        motors[1].set(TalonFXControlMode.PercentOutput, 
            pidOutput);  
    }

    public void setDesiredStateCancoder(SwerveModuleState desiredState) {
        SwerveModuleState state =
            SwerveModuleState.optimize(
                desiredState, 
                getCancoderAngle());

        double unitsVel = desiredState.speedMetersPerSecond / ConversionConstants.CTRE_NATIVE_TO_MPS;
        motors[0].set(TalonFXControlMode.Velocity, unitsVel);

        SmartDashboard.putNumber("ANGLESTATE", desiredState.angle.getRadians());

        double ticks = ConversionConstants.CHANGED_CTRE_TICKS_PER_REV;

        double setpoint =
            desiredState.angle.getRadians() / (2*Math.PI) * ticks;
        // setpoint = desiredState.angle.getRadians() / (2*Math.PI) * ConversionConstants.CTRE_TICKS_PER_REV;

        double current =
            getCancoderAngle().getRadians() / (2*Math.PI) * ticks;

        SmartDashboard.putNumber("SETPOINT", setpoint);
        double pidOutput = MathUtil.clamp(rotPID.calculate(current, setpoint), -0.8, 0.8);
        SmartDashboard.putNumber("PID OUTPUT", pidOutput);
        motors[1].set(TalonFXControlMode.PercentOutput, 
            pidOutput);  
    }

    /**
     * set angle encoder to a double
     * @param i set position to this
     */
    public void resetAngleEncoder(double i) {
        motors[1].setSelectedSensorPosition(i);
    }

    public void resetTalonAngleByCancoderOffset(double i) {
        SmartDashboard.putNumber("abspos", cancoder.getAbsolutePosition());
        System.out.println(cancoder.getAbsolutePosition());
        resetAngleEncoder(i+cancoder.getAbsolutePosition());
    }

    /**
     * set drive sensor position to a double
     * @param i set position to this (raw ctre)
     */
    public void resetDriveEncoder(double i) {
        motors[0].setSelectedSensorPosition(i);
    }

    /**
     * update PID config of motors from PID Constants
     */
    public void updatePID() {
        motors[0].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
        motors[0].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
        motors[0].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
        motors[0].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);
        motors[0].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.IZONE, PIDConstants.TIMEOUT_MS);

        motors[1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
        motors[1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
        motors[1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
        motors[1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);
        motors[1].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.IZONE, PIDConstants.TIMEOUT_MS);
    }

    /**
     * change motor inversion from two booleans
     * @param drive inversion of drive motor
     * @param rot inversion of rotation motor
     */
    public void changeMotorInversion(boolean drive, boolean rot) {
        motors[0].setInverted(drive);
        motors[1].setInverted(rot);
    }

    /**
     * initialize motor configs
     */
    private void initializeMotors() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].configFactoryDefault();
            motors[i].set(ControlMode.PercentOutput, 0);
            motors[i].setNeutralMode(NeutralMode.Brake);
            motors[i].configNeutralDeadband(0.001);
        }
        motors[1].setInverted(m_RotInverted);
        motors[0].setInverted(m_driveInverted);
    }

    /**
     * initialize motor PID configs
     */
    private void initializeMotorsPID() {
        motors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PIDConstants.PID_LOOP_IDX, 
        PIDConstants.TIMEOUT_MS);
        motors[1].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PIDConstants.PID_LOOP_IDX, 
        PIDConstants.TIMEOUT_MS);
        
  
  
        /* Config the peak and nominal outputs */
        motors[0].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
        motors[0].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
        motors[0].configPeakOutputForward(PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
        motors[0].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_VELOCITY.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
  
        motors[1].configNominalOutputForward(0, PIDConstants.TIMEOUT_MS);
        motors[1].configNominalOutputReverse(0, PIDConstants.TIMEOUT_MS);
        motors[1].configPeakOutputForward(PIDConstants.DRIVE_GAINS_POSITION.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
        motors[1].configPeakOutputReverse(-PIDConstants.DRIVE_GAINS_POSITION.PEAK_OUTPUT, PIDConstants.TIMEOUT_MS);
  
        /* Config the Velocity closed loop gains in slot0 */
        motors[0].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.F, PIDConstants.TIMEOUT_MS);
        motors[0].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.P, PIDConstants.TIMEOUT_MS);
        motors[0].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.I, PIDConstants.TIMEOUT_MS);
        motors[0].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.D, PIDConstants.TIMEOUT_MS);
        motors[0].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_VELOCITY.IZONE, PIDConstants.TIMEOUT_MS);
  
        motors[1].config_kF(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.F, PIDConstants.TIMEOUT_MS);
        motors[1].config_kP(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.P, PIDConstants.TIMEOUT_MS);
        motors[1].config_kI(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.I, PIDConstants.TIMEOUT_MS);
        motors[1].config_kD(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.D, PIDConstants.TIMEOUT_MS);
        motors[1].config_IntegralZone(PIDConstants.PID_LOOP_IDX, PIDConstants.DRIVE_GAINS_POSITION.IZONE, PIDConstants.TIMEOUT_MS);
  
        /* Hopefully make not continous */
        motors[1].configFeedbackNotContinuous(true, PIDConstants.TIMEOUT_MS);
        motors[1].configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
        motors[1].configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

    }
    private void initializeCancoder() {
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        cancoder.setPositionToAbsolute();
        cancoder.configSensorDirection(true);
    }
}
