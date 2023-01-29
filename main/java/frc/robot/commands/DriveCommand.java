package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.constants.RobotConstants.Swerve;
import frc.robot.subsystems.AdjustableTelemetry;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Joystick controller;
    private final AdjustableTelemetry tele;
    private boolean defaultFieldRelative;

    private double vx = 0;
    private double vy = 0;
    private double rad = 0;
    private int toggleModule = 0;

    SlewRateLimiter filter = new SlewRateLimiter(Swerve.Limits.slew);
    
    /**
     * Default command for driving
     * @param drive drive subsystem
     * @param controller drive controller
     * @param tele adjustable telemetry
     * @param defaultFieldRelative if calculating with field relative
     */
    public DriveCommand(
        DriveSubsystem drive, 
        AdjustableTelemetry tele,
        Joystick controller,
        boolean defaultFieldRelative
        ) {
        this.drive = drive;
        this.tele = tele;
        this.controller = controller;
        this.defaultFieldRelative = defaultFieldRelative;

        addRequirements(drive);
        initializeTelemetry();
    }

    private double controllerInput(double input, double deadband, double scaler) {
        return filter.calculate(MathUtil.applyDeadband(input, deadband)) * scaler;
    }

    /**
     * local driving function
     */
    private void drive() {
        if (!controller.getRawButton(2)) {
            vx = controllerInput(controller.getY(), Swerve.Limits.vxDeadband, Swerve.Limits.vx);
            vy = controllerInput(controller.getX(), Swerve.Limits.vyDeadband, Swerve.Limits.vy);
            rad = controllerInput(controller.getZ(), Swerve.Limits.radDeadband, Swerve.Limits.rad);
        } else {
            vx = controllerInput(controller.getY(), Swerve.Limits.vxDeadband, Swerve.Limits.vx * Swerve.Limits.reduced);
            vy = controllerInput(controller.getX(), Swerve.Limits.vyDeadband, Swerve.Limits.vy * Swerve.Limits.reduced);
            rad = controllerInput(controller.getZ(), Swerve.Limits.radDeadband, Swerve.Limits.rad * Swerve.Limits.reduced);
        }

        if (controller.getRawButton(4))
            toggleModule = toggleModule >= 1 ? 0 : toggleModule++;

        Boolean isFieldRelative = defaultFieldRelative ? !controller.getRawButton(6) : controller.getRawButton(6);
        // if default is true, then when button is not pressed, its field relative, if false, then opposite
        drive.setSpeeds(rad, vx, vy, isFieldRelative);
        
        drive.updateModuleStates();
        if (controller.getRawButton(5)) 
            drive.driveFromStopped(); 
        else 
            drive.drive();
        
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
    public void setDefaultFieldRelative(boolean i) {
        defaultFieldRelative = i;
    }

    @Override
    public void execute() {
        tele.updateDriveLimits();
        updateTelemetry();
        drive();
    }
}
