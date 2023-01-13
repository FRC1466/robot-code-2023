package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

import java.lang.Math;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private final TelemetrySubsystem m_tele;
    private boolean m_isFieldRelative;
    private double vx = 0;
    private double vy = 0;
    private double rot = 0;
    private int PID_iter = 0;
    private int m_toggleModule = 0;
    
    /**
     * Default command for driving
     * @param subsystem drive subsystem
     * @param controller drive controller
     */
    public DriveCommand(
        DriveSubsystem subsystem, 
        TelemetrySubsystem tele,
        XboxController controller,
        boolean isFieldRelative
        ) {
        m_drive = subsystem;
        m_tele = tele;
        addRequirements(m_drive);
        m_controller = controller;
        m_isFieldRelative = isFieldRelative;

         /* m_drive.resetAngleByCancoderOffset(
            new double[] {
                SmartDashboard.getNumber("0 cc", 0.0),
                SmartDashboard.getNumber("1 cc", 0.0),
                SmartDashboard.getNumber("2 cc", 0.0),
                SmartDashboard.getNumber("3 cc", 0.0),
            }
        ); */
        SmartDashboard.putNumber("vx limit", DriveConstants.LIMIT_VX);
        SmartDashboard.putNumber("vy limit", DriveConstants.LIMIT_VY); 
        SmartDashboard.putNumber("rot limit", DriveConstants.LIMIT_ROT);
        
    }

    /**
     * local driving function
     */
    private void m_drive() {
        vx = m_controller.getLeftX() * DriveConstants.LIMIT_VX;
        vy = -m_controller.getLeftY() * DriveConstants.LIMIT_VY;
        rot = -m_controller.getRightX() * DriveConstants.LIMIT_ROT;

        if (m_controller.getAButtonPressed()) {
            if(m_toggleModule >=4) {
                m_toggleModule = 0;
            } else {
                m_toggleModule++;
            }
            SmartDashboard.putNumber("togglemodule", m_toggleModule);
        }

            vx = 0;
        }

        if (!(Math.abs(vy) > 0.15)) {
            vy = 0;
        }

        if (!(Math.abs(rot) > 0.16)) {
            rot = 0;
        }

        if (m_isFieldRelative) {
            m_drive.updateSpeedsFieldRelative(rot, vx, vy);
        } else {
            m_drive.updateSpeeds(rot, vx, vy);
        }
        
        m_drive.updateModuleStates();
        switch (m_toggleModule) {
            case 0:
            m_drive.drive();
                break;
            case 1:
            m_drive.driveSpecificModule(1);
                break;
            case 2:
            m_drive.driveSpecificModule(2);
                break;
            case 3:
            m_drive.driveSpecificModule(3);
                break;
            case 4:
            m_drive.driveSpecificModule(4);
                break;
            default:
                break;
        }

        if (m_controller.getBButtonPressed()) {
            switch (m_toggleModule) {
                case 1:
                m_drive.resetSpecificAngleEncoder(1);
                    break;
                case 2:
                m_drive.resetSpecificAngleEncoder(2);
                    break;
                case 3:
                m_drive.resetSpecificAngleEncoder(3);
                    break;
                case 4:
                m_drive.resetSpecificAngleEncoder(4);
                    break;
                default:
                    break;
            }}


        
    }

    /**
     * Update PID system from SmartDashboard for quick and easy tuning
     */
    private void updatePID() {
        m_tele.updatePIDConstants();
        m_drive.updatePIDConfigs();
        m_drive.setModulePositionPID(
            PIDConstants.DRIVE_GAINS_POSITION.P,
            PIDConstants.DRIVE_GAINS_POSITION.I,
            PIDConstants.DRIVE_GAINS_POSITION.D);
    }

    private void debuggingUpdate() {
        m_tele.updateEncoders();
        m_tele.setModuleInversion();
        m_drive.updateModuleInversion();
        m_tele.setCodeDebugStates();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);
    }
    

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        m_tele.updateDriveLimits();
        m_drive();
        updateSmartDashboard();
        if (PID_iter*20 > 5000) { // 5000ms PID update time
            updatePID();
            debuggingUpdate();
            PID_iter = 0;
        }
        PID_iter++;
        
    }
}
