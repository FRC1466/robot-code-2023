package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AdjustableTelemetry;

import java.lang.Math;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private final AdjustableTelemetry m_tele;
    private boolean m_isFieldRelative;
    private double vx = 0;
    private double vy = 0;
    private double rad = 0;
    private int PID_iter = 0;
    private int m_toggleModule = 0;
    
    /**
     * Default command for driving
     * @param subsystem drive subsystem
     * @param controller drive controller
     */
    public DriveCommand(
        DriveSubsystem subsystem, 
        AdjustableTelemetry tele,
        XboxController controller,
        boolean isFieldRelative
        ) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_tele = tele;
        m_controller = controller;
        m_isFieldRelative = isFieldRelative;

        initializeTelemetry();
    }

    /**
     * local driving function
     */
    private void m_drive() {
        vx = Math.abs(m_controller.getLeftX())>0.07 ? m_controller.getLeftX() * DriveConstants.LIMIT_VX : 0;
        vy = Math.abs(m_controller.getLeftY())>0.07 ? m_controller.getLeftY() * DriveConstants.LIMIT_VY : 0;
        rad = Math.abs(m_controller.getRightX())>0.05 ? m_controller.getRightX() * DriveConstants.LIMIT_RAD : 0;

        if (m_controller.getYButtonPressed())
            m_toggleModule = m_toggleModule >= 1 ? 0 : m_toggleModule++;

        if (m_isFieldRelative)
            m_drive.setSpeedsFieldRelative(rad, vx, vy);
        else
            m_drive.setSpeeds(rad, vx, vy);
        
        m_drive.updateModuleStates();
        switch (m_toggleModule) {
            case 0:
                if (m_controller.getLeftTriggerAxis() > 0.8) 
                    m_drive.driveFromStopped(); 
                else 
                    m_drive.drive();
                break;
            case 1:
                m_drive.drivePosSpecificModule(m_controller.getRightTriggerAxis());
                break;
            default:
                break;
        }
        
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

    private GenericEntry vxEntry;
    private GenericEntry vyEntry;
    private GenericEntry radEntry;
    private GenericEntry moduleToggleEntry;
    /**
     * initialize telemetry
     */
    private void initializeTelemetry() {
        ShuffleboardTab teleTab = Shuffleboard.getTab("Telemetry");
        ShuffleboardLayout driveLayout = teleTab
            .getLayout("drive", BuiltInLayouts.kList)
            .withSize(1, 3);
        vxEntry = driveLayout.add("vx", vx).getEntry();
        vyEntry = driveLayout.add("vy", vy).getEntry();
        radEntry = driveLayout.add("rad", rad).getEntry();
        moduleToggleEntry = driveLayout.add("module toggle", m_toggleModule).getEntry();
    }

    /**
     * update telemetry
     */
    private void updateTelemetry() {
        vxEntry.setDouble(vx);
        vyEntry.setDouble(vy);
        radEntry.setDouble(rad);
        moduleToggleEntry.setDouble(m_toggleModule);
    }

    /**
     * set field relative
     * @param i false or true
     */
    public void setFieldRelative(boolean i) {
        m_isFieldRelative = i;
    }
    

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        m_tele.updateDriveLimits();
        updateTelemetry();
        m_drive();
        
        if (PID_iter*20 > 5000) { // 5000ms PID update time
            updatePID();
            debuggingUpdate();
            PID_iter = 0;
        }
        PID_iter++;
        
    }
}
