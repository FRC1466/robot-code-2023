package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.PIDConstants;

public class TelemetrySubsystem {
    private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    public TelemetrySubsystem() {
        initializePIDUpdate();
        initializeEncoderUpdates();
        initializeDriveLimits();
        initializeModuleInversion();
        initializeCodeDebugStates();
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
        PIDConstants.DRIVE_GAINS_POSITION.P = m_P_pos.getDouble(0);
        PIDConstants.DRIVE_GAINS_POSITION.I = m_I_pos.getDouble(0);
        PIDConstants.DRIVE_GAINS_POSITION.D = m_D_pos.getDouble(0);
        PIDConstants.DRIVE_GAINS_POSITION.F = m_F_pos.getDouble(0);
        PIDConstants.DRIVE_GAINS_POSITION.IZONE = m_Izone_pos.getDouble(0);
    
        PIDConstants.DRIVE_GAINS_VELOCITY.P = m_P_vel.getDouble(0);
        PIDConstants.DRIVE_GAINS_VELOCITY.I = m_I_vel.getDouble(0);
        PIDConstants.DRIVE_GAINS_VELOCITY.D = m_D_vel.getDouble(0);
        PIDConstants.DRIVE_GAINS_VELOCITY.F = m_F_vel.getDouble(0);
        PIDConstants.DRIVE_GAINS_VELOCITY.IZONE = m_Izone_vel.getDouble(0);

        // AutoConstants.TRANSLATION_CONTROLLER.P = m_P_translation.getDouble(0);
        // AutoConstants.TRANSLATION_CONTROLLER.I = m_I_translation.getDouble(0);
        // AutoConstants.TRANSLATION_CONTROLLER.D = m_D_translation.getDouble(0);

        // AutoConstants.THETA_CONTROLLER.P = m_P_theta.getDouble(0);
        // AutoConstants.THETA_CONTROLLER.I = m_I_theta.getDouble(0);
        // AutoConstants.THETA_CONTROLLER.D = m_D_theta.getDouble(0);
    }
    
    private void initializePIDUpdate() {
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");
        System.out.println("ASJDGKASJDGHELPPSWORK");

        ShuffleboardLayout positionPID = tuningTab
            .getLayout("Position PID", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(1, 5);

        m_P_pos = positionPID.add("P_pos", PIDConstants.DRIVE_GAINS_POSITION.P).getEntry();
        m_I_pos = positionPID.add("I_pos", PIDConstants.DRIVE_GAINS_POSITION.I).getEntry();
        m_D_pos = positionPID.add("D_pos", PIDConstants.DRIVE_GAINS_POSITION.D).getEntry();
        m_F_pos = positionPID.add("F_pos", PIDConstants.DRIVE_GAINS_POSITION.F).getEntry();
        m_Izone_pos = positionPID.add("Izone_pos", PIDConstants.DRIVE_GAINS_POSITION.IZONE).getEntry();

        ShuffleboardLayout velocityPID = tuningTab
            .getLayout("Velocity PID", BuiltInLayouts.kList)
            .withPosition(1, 0)
            .withSize(1, 5);

        m_P_vel = velocityPID.add("P_vel", PIDConstants.DRIVE_GAINS_VELOCITY.P).getEntry();
        m_I_vel = velocityPID.add("I_vel", PIDConstants.DRIVE_GAINS_VELOCITY.I).getEntry();
        m_D_vel = velocityPID.add("D_vel", PIDConstants.DRIVE_GAINS_VELOCITY.D).getEntry();
        m_F_vel = velocityPID.add("F_vel", PIDConstants.DRIVE_GAINS_VELOCITY.F).getEntry();
        m_Izone_vel = velocityPID.add("Izone_vel", PIDConstants.DRIVE_GAINS_VELOCITY.IZONE).getEntry();

        // ShuffleboardLayout autoPID = tuningTab
        //     .getLayout("Auto PID", BuiltInLayouts.kList)
        //     .withSize(1, 6);

        // m_P_translation = autoPID.add("P_translation", AutoConstants.TRANSLATION_CONTROLLER.P).getEntry();
        // m_I_translation = autoPID.add("I_translation", AutoConstants.TRANSLATION_CONTROLLER.I).getEntry();
        // m_D_translation = autoPID.add("D_translation", AutoConstants.TRANSLATION_CONTROLLER.D).getEntry();
        // m_P_theta = autoPID.add("P_theta", AutoConstants.THETA_CONTROLLER.P).getEntry();
        // m_I_theta = autoPID.add("I_theta", AutoConstants.THETA_CONTROLLER.I).getEntry();
        // m_D_theta = autoPID.add("D_theta", AutoConstants.THETA_CONTROLLER.D).getEntry();
    } 

    private GenericEntry frontLeftCancoder;
    private GenericEntry frontRightCancoder;
    private GenericEntry backLeftCancoder;
    private GenericEntry backRightCancoder;
    private GenericEntry integratedFalconFullRotation;

    private void initializeEncoderUpdates() {
        ShuffleboardLayout Encoders = tuningTab
            .getLayout("Encoders", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        frontLeftCancoder = Encoders.add("frontLeftCancoderOffset", DriveConstants.FRONTLEFT_OFFSET).getEntry();
        frontRightCancoder = Encoders.add("frontRightCancoderOffset", DriveConstants.FRONTRIGHT_OFFSET).getEntry();
        backLeftCancoder = Encoders.add("backLeftCancoderOffset", DriveConstants.BACKLEFT_OFFSET).getEntry();
        backRightCancoder = Encoders.add("backRightCancoderOffset", DriveConstants.BACKRIGHT_OFFSET).getEntry();
        integratedFalconFullRotation = Encoders.add("integratedFalconFullRotation", ConversionConstants.CTRE_TICKS_PER_REV).getEntry();
    }

    public void updateEncoders() {
        DriveConstants.FRONTLEFT_OFFSET = frontLeftCancoder.getDouble(DriveConstants.FRONTLEFT_OFFSET);
        DriveConstants.FRONTRIGHT_OFFSET = frontRightCancoder.getDouble(DriveConstants.FRONTRIGHT_OFFSET);
        DriveConstants.BACKLEFT_OFFSET = backLeftCancoder.getDouble(DriveConstants.BACKLEFT_OFFSET);
        DriveConstants.BACKRIGHT_OFFSET = backRightCancoder.getDouble(DriveConstants.BACKRIGHT_OFFSET);
        ConversionConstants.CHANGED_CTRE_TICKS_PER_REV = integratedFalconFullRotation.getDouble(ConversionConstants.CTRE_TICKS_PER_REV);
    }

    private GenericEntry vxLimit;
    private GenericEntry vyLimit;
    private GenericEntry rotLimit;
    private GenericEntry clampLimit;

    private void initializeDriveLimits() {
        ShuffleboardLayout driveLimits = tuningTab
            .getLayout("Drive Limits", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        vxLimit = driveLimits.add("vxLimit", DriveConstants.LIMIT_VX).getEntry();
        vyLimit = driveLimits.add("vyLimit", DriveConstants.LIMIT_VY).getEntry();
        rotLimit = driveLimits.add("rotLimit", DriveConstants.LIMIT_ROT).getEntry();
        clampLimit = driveLimits.add("clampLimit", DriveConstants.LIMIT_PID_CLAMP).getEntry();
    }

    public void updateDriveLimits() {
        DriveConstants.LIMIT_VX = vxLimit.getDouble(DriveConstants.LIMIT_VX);
        DriveConstants.LIMIT_VY = vyLimit.getDouble(DriveConstants.LIMIT_VY);
        DriveConstants.LIMIT_ROT = rotLimit.getDouble(DriveConstants.LIMIT_ROT);
        DriveConstants.LIMIT_PID_CLAMP = clampLimit.getDouble((DriveConstants.LIMIT_PID_CLAMP));
    }

    private GenericEntry frontLeftDriveInvert;
    private GenericEntry frontRightDriveInvert;
    private GenericEntry backLeftDriveInvert;
    private GenericEntry backRightDriveInvert;
    private GenericEntry frontLeftRotInvert;
    private GenericEntry frontRightRotInvert;
    private GenericEntry backLeftRotInvert;
    private GenericEntry backRightRotInvert;

    private void initializeModuleInversion() {
        ShuffleboardLayout driveInvert = tuningTab
            .getLayout("driveInvert", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        frontLeftDriveInvert = 
            driveInvert.add("frontLeftDriveInvert", DriveConstants.FRONTLEFT_DRIVEINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        frontRightDriveInvert = 
            driveInvert.add("frontRightDriveInvert", DriveConstants.FRONTRIGHT_DRIVEINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        backLeftDriveInvert = 
            driveInvert.add("backLeftDriveInvert", DriveConstants.BACKLEFT_DRIVEINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        backRightDriveInvert = 
            driveInvert.add("backRightDriveInvert", DriveConstants.BACKRIGHT_DRIVEINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

        ShuffleboardLayout rotInvert = tuningTab
            .getLayout("rotInvert", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        frontLeftRotInvert = 
            rotInvert.add("frontLeftDriveInvert", DriveConstants.FRONTLEFT_ROTINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        frontRightRotInvert = rotInvert.add("frontRightDriveInvert", DriveConstants.FRONTRIGHT_ROTINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        backLeftRotInvert = rotInvert.add("backLeftDriveInvert", DriveConstants.BACKLEFT_ROTINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
        backRightRotInvert = rotInvert.add("backRightDriveInvert", DriveConstants.BACKRIGHT_ROTINVERT)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    }

    public void setModuleInversion() {
        DriveConstants.FRONTLEFT_DRIVEINVERT = frontLeftDriveInvert.getBoolean(DriveConstants.FRONTLEFT_DRIVEINVERT);
        DriveConstants.FRONTRIGHT_DRIVEINVERT = frontRightDriveInvert.getBoolean(DriveConstants.FRONTRIGHT_DRIVEINVERT);
        DriveConstants.BACKLEFT_DRIVEINVERT = backLeftDriveInvert.getBoolean(DriveConstants.BACKLEFT_DRIVEINVERT);
        DriveConstants.BACKRIGHT_DRIVEINVERT = backRightDriveInvert.getBoolean(DriveConstants.BACKRIGHT_DRIVEINVERT);

        DriveConstants.FRONTLEFT_ROTINVERT = frontLeftRotInvert.getBoolean(DriveConstants.FRONTLEFT_ROTINVERT);
        DriveConstants.FRONTRIGHT_ROTINVERT = frontRightRotInvert.getBoolean(DriveConstants.FRONTRIGHT_ROTINVERT);
        DriveConstants.BACKLEFT_ROTINVERT = backLeftRotInvert.getBoolean(DriveConstants.BACKLEFT_ROTINVERT);
        DriveConstants.BACKRIGHT_ROTINVERT = backRightRotInvert.getBoolean(DriveConstants.BACKRIGHT_ROTINVERT);
    }

    private GenericEntry usingCancoderPID;
    

    private void initializeCodeDebugStates() {
        ShuffleboardLayout debug = tuningTab
            .getLayout("debug", BuiltInLayouts.kList)
            .withSize(1, 4);

        usingCancoderPID = 
            debug.add("isUsingCancoderPID", DebugConstants.isUsingCancoderPID)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();

    }

    public void setCodeDebugStates() {
        DebugConstants.isUsingCancoderPID = usingCancoderPID.getBoolean(DebugConstants.isUsingCancoderPID);
    }

}
