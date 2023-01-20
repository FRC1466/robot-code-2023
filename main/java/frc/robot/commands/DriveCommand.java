package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.AdjustableTelemetry;

import java.lang.Math;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Joystick controller;
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
        Joystick controller,
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
        if (!controller.getRawButton(2)) {
            vx = Math.abs(controller.getX())>0.02 ? -controller.getX() * Swerve.Limits.vx : 0;
            vy = Math.abs(controller.getY())>0.02 ? controller.getY() * Swerve.Limits.vy : 0;
            rad = Math.abs(controller.getZ())>0.35 ? -controller.getZ() * Swerve.Limits.rad : 0;
            SmartDashboard.putNumber("Z", controller.getZ());
        } else {
            vx = Math.abs(controller.getX())>0.02 ? -controller.getX() * Swerve.Limits.vx * 0.3 : 0;
            vy = Math.abs(controller.getY())>0.02 ? controller.getY() * Swerve.Limits.vy * 0.3 : 0;
            rad = Math.abs(controller.getZ())>0.26 ? -controller.getZ() * Swerve.Limits.rad * 0.3 : 0;
        }

        if (controller.getRawButton(4))
            toggleModule = toggleModule >= 1 ? 0 : toggleModule++;

        if (isFieldRelative)
            drive.setSpeedsFieldRelative(rad, vx, vy);
        else
            drive.setSpeeds(rad, vx, vy);
        
        drive.updateModuleStates();
        drive.drive();
        // switch (toggleModule) {
        //     case 0:
        //         if (controller.getRawButton(5)) 
        //             drive.driveFromStopped(); 
        //         else 
        //             drive.drive();
        //         break;
        //     case 1:
        //         drive.drivePosSpecificModule(controller.getY());
        //         break;
        //     default:
        //         break;
        // }
        
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
        
        // if (pidIter*20 > 5000) { // 5000ms PID update time
        //     updatePID();
        //     pidIter = 0;
        // }
        // pidIter++;
    }
}
