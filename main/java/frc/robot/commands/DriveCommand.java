package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.AdjustableTelemetry;

import java.lang.Math;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final XboxController controller;
    private final AdjustableTelemetry tele;
    private boolean isFieldRelative;

    private double vx = 0;
    private double vy = 0;
    private double rad = 0;
    private int pidIter = 0;
    private int toggleModule = 0;
    
    /**
     * Default command for driving
     * @param drive drive subsystem
     * @param controller drive controller
     * @param tele adjustable telemetry
     * @param isFieldRelative if calculating with field relative
     */
    public DriveCommand(
        DriveSubsystem drive, 
        AdjustableTelemetry tele,
        XboxController controller,
        boolean isFieldRelative
        ) {
        this.drive = drive;
        this.tele = tele;
        this.controller = controller;
        this.isFieldRelative = isFieldRelative;

        addRequirements(drive);
        initializeTelemetry();
    }

    /**
     * local driving function
     */
    private void drive() {
        vx = Math.abs(controller.getLeftX())>0.07 ? controller.getLeftX() * Swerve.Limits.vx : 0;
        vy = Math.abs(controller.getLeftY())>0.07 ? controller.getLeftY() * Swerve.Limits.vy : 0;
        rad = Math.abs(controller.getRightX())>0.05 ? controller.getRightX() * Swerve.Limits.rad : 0;

        if (controller.getYButtonPressed())
            toggleModule = toggleModule >= 1 ? 0 : toggleModule++;

        if (isFieldRelative)
            drive.setSpeedsFieldRelative(rad, vx, vy);
        else
            drive.setSpeeds(rad, vx, vy);
        
        drive.updateModuleStates();
        switch (toggleModule) {
            case 0:
                if (controller.getLeftTriggerAxis() > 0.8) 
                    drive.driveFromStopped(); 
                else 
                    drive.drive();
                break;
            case 1:
                drive.drivePosSpecificModule(controller.getRightTriggerAxis());
                break;
            default:
                break;
        }
        
    }

    /**
     * Update PID system from SmartDashboard for quick and easy tuning
     */
    private void updatePID() {
        tele.updatePIDConstants();
        drive.updatePIDConfigs();
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
        moduleToggleEntry = driveLayout.add("module toggle", toggleModule).getEntry();
    }

    /**
     * update telemetry
     */
    private void updateTelemetry() {
        vxEntry.setDouble(vx);
        vyEntry.setDouble(vy);
        radEntry.setDouble(rad);
        moduleToggleEntry.setDouble(toggleModule);
    }

    /**
     * set field relative
     * @param i false or true
     */
    public void setFieldRelative(boolean i) {
        isFieldRelative = i;
    }

    @Override
    public void execute() {
        tele.updateDriveLimits();
        updateTelemetry();
        drive();
        
        if (pidIter*20 > 5000) { // 5000ms PID update time
            updatePID();
            pidIter = 0;
        }
        pidIter++;
    }
}
