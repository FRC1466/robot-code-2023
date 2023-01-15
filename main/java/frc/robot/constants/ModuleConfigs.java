package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.constants.Constants.Swerve;

public final class ModuleConfigs {

    public TalonFXConfiguration swerveAngleConfig;
    public TalonFXConfiguration swerveDriveConfig;
    public CANCoderConfiguration swerveCancoderConfig;

    /**
     * create a set of configs for a swerve module
     */
    public ModuleConfigs() {
        swerveAngleConfig = new TalonFXConfiguration();
        swerveDriveConfig = new TalonFXConfiguration();
        swerveCancoderConfig = new CANCoderConfiguration();

        swerveAngleConfig.nominalOutputForward = 0;
        swerveAngleConfig.nominalOutputReverse = 0;
        swerveAngleConfig.peakOutputForward = Swerve.driveGainsPosition.peakOutput;
        swerveAngleConfig.peakOutputReverse = -Swerve.driveGainsPosition.peakOutput;
        swerveAngleConfig.slot0.kP = Swerve.driveGainsPosition.P;
        swerveAngleConfig.slot0.kI = Swerve.driveGainsPosition.I;
        swerveAngleConfig.slot0.kD = Swerve.driveGainsPosition.D;
        swerveAngleConfig.slot0.kF = Swerve.driveGainsPosition.F;
        swerveAngleConfig.slot0.integralZone = Swerve.driveGainsPosition.integralZone;
        swerveAngleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        swerveDriveConfig.nominalOutputForward = 0;
        swerveDriveConfig.nominalOutputReverse = 0;
        swerveDriveConfig.peakOutputForward = Swerve.driveGainsVelocity.peakOutput;
        swerveDriveConfig.peakOutputReverse = -Swerve.driveGainsVelocity.peakOutput;
        swerveDriveConfig.slot0.kP = Swerve.driveGainsVelocity.P;
        swerveDriveConfig.slot0.kI = Swerve.driveGainsVelocity.I;
        swerveDriveConfig.slot0.kD = Swerve.driveGainsVelocity.D;
        swerveDriveConfig.slot0.kF = Swerve.driveGainsVelocity.F;
        swerveDriveConfig.slot0.integralZone = Swerve.driveGainsVelocity.integralZone;
        swerveDriveConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        swerveCancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        swerveCancoderConfig.sensorDirection = true;

    }

    
}
