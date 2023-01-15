package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.ModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Robot;
import frc.robot.constants.Constants.Swerve;

public class SwerveModule {
    public int moduleNumber;
    private WPI_TalonFX angleMotor;
    private WPI_TalonFX driveMotor;
    private WPI_CANCoder cancoder;
    private boolean angleInvert;
    private boolean driveInvert;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private Rotation2d driftOffset;
    
    /**
     * Initialize a Swerve Module
     */
    public SwerveModule(
        int moduleID,
        SwerveModuleConstants moduleConstants
    ) {
        this.angleInvert = moduleConstants.angleInvert;
        this.driveInvert = moduleConstants.driveInvert;
        this.angleOffset = moduleConstants.angleOffset;
        this.moduleNumber = moduleID;

        cancoder = new WPI_CANCoder(moduleConstants.cancoderID);
        configCancoder();

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        lastAngle = getState().angle;
        driftOffset = new Rotation2d();
    }

    /**
     * @return rotation of falcon motor
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), 
        Swerve.angleGearRatio));
    }

    /**
     * get current state of swerve module
     * @return SwerveModuleState created from current encoder layouts
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Swerve.wheelCircumference, Swerve.driveGearRatio),
            getAngle()
            );
    }

    /**
     * @return current module positions of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Swerve.wheelCircumference, Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public Rotation2d getCancoderAngle() 
    {
        return Rotation2d.fromDegrees(
            cancoder.getAbsolutePosition()
            );
    }


    /**
     * set state of swerve module given a SwerveModuleState
     * @param desiredState the SwerveModuleState to set to
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = ModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    /**
     * set speed of swerve module given a SwerveModuleState
     * @param desiredState the SwerveModuleState to set to
     */
    public void setSpeed(SwerveModuleState desiredState) {
        double sp = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Swerve.wheelCircumference, Swerve.driveGearRatio);
        driveMotor.set(TalonFXControlMode.Velocity, sp);
    }

    /**
     * set angle of swerve module given a SwerveModuleState
     * @param desiredState the SwerveModuleState to set to
     */
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (4 * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        driveMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.plus(driftOffset).getDegrees(), Swerve.angleGearRatio));
        lastAngle = angle;
    }

    /**
     * reset falcon angles to the absolute position of encoder (incl offset)
     */
    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCancoderAngle().getDegrees() - angleOffset.getDegrees(), Swerve.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * set drive position to a degree
     * @param i position in degrees
     */
    public void setDrivePosition(double i) {
        double pos = Conversions.degreesToFalcon(i, Swerve.driveGearRatio);
        driveMotor.set(TalonFXControlMode.Position, pos);
    }

    /**
     * set drive sensor position to a double
     * @param i set position to this (raw ctre)
     */
    public void setDriveEncoder(double i) {
        driveMotor.setSelectedSensorPosition(i);
    }

    /**
     * update PID config of motors from PID Constants
     */
    public void updatePID() {
        driveMotor.config_kF(Swerve.pidLoopIdx, Swerve.driveGainsVelocity.F, Swerve.timeoutMS);
        driveMotor.config_kP(Swerve.pidLoopIdx, Swerve.driveGainsVelocity.P, Swerve.timeoutMS);
        driveMotor.config_kI(Swerve.pidLoopIdx, Swerve.driveGainsVelocity.I, Swerve.timeoutMS);
        driveMotor.config_kD(Swerve.pidLoopIdx, Swerve.driveGainsVelocity.D, Swerve.timeoutMS);
        driveMotor.config_IntegralZone(Swerve.pidLoopIdx, Swerve.driveGainsVelocity.integralZone, Swerve.timeoutMS);

        angleMotor.config_kF(Swerve.pidLoopIdx, Swerve.driveGainsPosition.F, Swerve.timeoutMS);
        angleMotor.config_kP(Swerve.pidLoopIdx, Swerve.driveGainsPosition.P, Swerve.timeoutMS);
        angleMotor.config_kI(Swerve.pidLoopIdx, Swerve.driveGainsPosition.I, Swerve.timeoutMS);
        angleMotor.config_kD(Swerve.pidLoopIdx, Swerve.driveGainsPosition.D, Swerve.timeoutMS);
        angleMotor.config_IntegralZone(Swerve.pidLoopIdx, Swerve.driveGainsPosition.integralZone, Swerve.timeoutMS);
    }

    /**
     * change motor inversion from two booleans
     * @param drive inversion of drive motor
     * @param rot inversion of rotation motor
     */
    public void setMotorInversion(boolean drive, boolean rot) {
        driveMotor.setInverted(drive);
        angleMotor.setInverted(rot);
    }

    private void configCancoder() {
        cancoder.configFactoryDefault();
        cancoder.configAllSettings(Robot.moduleConfigs.swerveCancoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Swerve.pidLoopIdx, 
        Swerve.timeoutMS);
        angleMotor.configAllSettings(Robot.moduleConfigs.swerveAngleConfig);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configNeutralDeadband(0.001);
        angleMotor.setInverted(angleInvert);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Swerve.pidLoopIdx, 
        Swerve.timeoutMS);
        driveMotor.configAllSettings(Robot.moduleConfigs.swerveDriveConfig);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configNeutralDeadband(0.001);
        driveMotor.setInverted(driveInvert);
    }
}
