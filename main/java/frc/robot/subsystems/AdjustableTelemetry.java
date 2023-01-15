package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.Swerve;

public class AdjustableTelemetry {
    private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    /**
     * initializes the Telemetry class
     */
    public AdjustableTelemetry() {
        initializePIDUpdate();
        initializeDriveLimits();
    }

    private GenericEntry m_P_pos;
    private GenericEntry m_I_pos;
    private GenericEntry m_D_pos;
    private GenericEntry m_F_pos;
    private GenericEntry m_Izone_pos;
    private GenericEntry m_P_vel;
    private GenericEntry m_I_vel;
    private GenericEntry m_D_vel;
    private GenericEntry m_F_vel;
    private GenericEntry m_Izone_vel;
    private GenericEntry m_P_translation;
    private GenericEntry m_I_translation;
    private GenericEntry m_D_translation;
    private GenericEntry m_P_theta;
    private GenericEntry m_I_theta;
    private GenericEntry m_D_theta;

    public void updatePIDConstants() {
        Swerve.driveGainsPosition.P = m_P_pos.getDouble(0);
        Swerve.driveGainsPosition.I = m_I_pos.getDouble(0);
        Swerve.driveGainsPosition.D = m_D_pos.getDouble(0);
        Swerve.driveGainsPosition.F = m_F_pos.getDouble(0);
        Swerve.driveGainsPosition.integralZone = m_Izone_pos.getDouble(0);
    
        Swerve.driveGainsVelocity.P = m_P_vel.getDouble(0);
        Swerve.driveGainsVelocity.I = m_I_vel.getDouble(0);
        Swerve.driveGainsVelocity.D = m_D_vel.getDouble(0);
        Swerve.driveGainsVelocity.F = m_F_vel.getDouble(0);
        Swerve.driveGainsVelocity.integralZone = m_Izone_vel.getDouble(0);

        AutoConstants.translationController.P = m_P_translation.getDouble(0);
        AutoConstants.translationController.I = m_I_translation.getDouble(0);
        AutoConstants.translationController.D = m_D_translation.getDouble(0);

        AutoConstants.thetaController.P = m_P_theta.getDouble(0);
        AutoConstants.thetaController.I = m_I_theta.getDouble(0);
        AutoConstants.thetaController.D = m_D_theta.getDouble(0);
    }
    
    private void initializePIDUpdate() {

        ShuffleboardLayout positionPID = tuningTab
            .getLayout("Position PID", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(1, 5);

        m_P_pos = positionPID.add("P_pos", Swerve.driveGainsPosition.P).getEntry();
        m_I_pos = positionPID.add("I_pos", Swerve.driveGainsPosition.I).getEntry();
        m_D_pos = positionPID.add("D_pos", Swerve.driveGainsPosition.D).getEntry();
        m_F_pos = positionPID.add("F_pos", Swerve.driveGainsPosition.F).getEntry();
        m_Izone_pos = positionPID.add("Izone_pos", Swerve.driveGainsPosition.integralZone).getEntry();

        ShuffleboardLayout velocityPID = tuningTab
            .getLayout("Velocity PID", BuiltInLayouts.kList)
            .withPosition(1, 0)
            .withSize(1, 5);

        m_P_vel = velocityPID.add("P_vel", Swerve.driveGainsVelocity.P).getEntry();
        m_I_vel = velocityPID.add("I_vel", Swerve.driveGainsVelocity.I).getEntry();
        m_D_vel = velocityPID.add("D_vel", Swerve.driveGainsVelocity.D).getEntry();
        m_F_vel = velocityPID.add("F_vel", Swerve.driveGainsVelocity.F).getEntry();
        m_Izone_vel = velocityPID.add("Izone_vel", Swerve.driveGainsVelocity.integralZone).getEntry();

        ShuffleboardLayout autoPID = tuningTab
            .getLayout("Auto PID", BuiltInLayouts.kList)
            .withSize(1, 6);

        m_P_translation = autoPID.add("P_translation", AutoConstants.translationController.P).getEntry();
        m_I_translation = autoPID.add("I_translation", AutoConstants.translationController.I).getEntry();
        m_D_translation = autoPID.add("D_translation", AutoConstants.translationController.D).getEntry();
        m_P_theta = autoPID.add("P_theta", AutoConstants.thetaController.P).getEntry();
        m_I_theta = autoPID.add("I_theta", AutoConstants.thetaController.I).getEntry();
        m_D_theta = autoPID.add("D_theta", AutoConstants.thetaController.D).getEntry();
    } 

    private GenericEntry vxLimit;
    private GenericEntry vyLimit;
    private GenericEntry rotLimit;

    private void initializeDriveLimits() {
        ShuffleboardLayout driveLimits = tuningTab
            .getLayout("Drive Limits", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        vxLimit = driveLimits.add("vxLimit", Swerve.Limits.vx).getEntry();
        vyLimit = driveLimits.add("vyLimit", Swerve.Limits.vy).getEntry();
        rotLimit = driveLimits.add("rotLimit", Swerve.Limits.rad).getEntry();
    }

    public void updateDriveLimits() {
        Swerve.Limits.vx = vxLimit.getDouble(Swerve.Limits.vx);
        Swerve.Limits.vy = vyLimit.getDouble(Swerve.Limits.vy);
        Swerve.Limits.rad = rotLimit.getDouble(Swerve.Limits.rad);
    }

}
