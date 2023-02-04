package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.OIConstants;
import frc.robot.subsystems.AdjustableTelemetry;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final Joystick controller;
  private final AdjustableTelemetry tele;
  private boolean defaultFieldRelative;

  private double vx = 0;
  private double vy = 0;
  private double rad = 0;
  private int toggleModule = 0;

  private final SlewRateLimiter filterX = new SlewRateLimiter(OIConstants.InputLimits.slew);
  private final SlewRateLimiter filterY = new SlewRateLimiter(OIConstants.InputLimits.slew);
  private final SlewRateLimiter filterZ = new SlewRateLimiter(OIConstants.InputLimits.slew);

  private final Debouncer debouncer =
      new Debouncer(OIConstants.InputLimits.debounce, Debouncer.DebounceType.kBoth);

  /**
   * Default command for driving
   *
   * @param drive drive subsystem
   * @param controller drive controller
   * @param tele adjustable telemetry
   * @param defaultFieldRelative if calculating with field relative
   */
  public DriveCommand(
      DriveSubsystem drive,
      AdjustableTelemetry tele,
      Joystick controller,
      boolean defaultFieldRelative) {
    this.drive = drive;
    this.tele = tele;
    this.controller = controller;
    this.defaultFieldRelative = defaultFieldRelative;

    addRequirements(drive);
    initializeTelemetry();
  }

  private double controllerInput(
      double input, double deadband, double scalar, SlewRateLimiter limiter) {
    input = MathUtil.applyDeadband(input, deadband);
    // var output = debouncer.calculate(input > 0) ? input : 0;
    return limiter.calculate(input) * scalar;
    // return input * scalar;
  }

  /** local driving function */
  private void drive() {
    if (!controller.getRawButton(2)) {
      vx =
          controllerInput(
              controller.getY(),
              OIConstants.InputLimits.vxDeadband,
              OIConstants.InputLimits.vx,
              filterX);
      vy =
          controllerInput(
              controller.getX(),
              OIConstants.InputLimits.vyDeadband,
              OIConstants.InputLimits.vy,
              filterY);
      rad =
          controllerInput(
              controller.getZ(),
              OIConstants.InputLimits.radDeadband,
              OIConstants.InputLimits.rad,
              filterZ);
    } else {
      vx =
          controllerInput(
              controller.getY(),
              OIConstants.InputLimits.vxDeadband,
              OIConstants.InputLimits.vx * OIConstants.InputLimits.reduced,
              filterX);
      vy =
          controllerInput(
              controller.getX(),
              OIConstants.InputLimits.vyDeadband,
              OIConstants.InputLimits.vy * OIConstants.InputLimits.reduced,
              filterY);
      rad =
          controllerInput(
              controller.getZ(),
              OIConstants.InputLimits.radDeadband,
              OIConstants.InputLimits.rad * OIConstants.InputLimits.reduced,
              filterZ);
    }

    if (controller.getRawButton(4)) toggleModule = toggleModule >= 1 ? 0 : toggleModule++;

    Boolean isFieldRelative =
        defaultFieldRelative ? !controller.getRawButton(6) : controller.getRawButton(6);
    drive.setSpeeds(rad, vx, vy, isFieldRelative);

    drive.updateModuleStates();
    if (controller.getRawButton(5)) drive.driveFromStopped();
    else drive.drive();
  }

  private GenericEntry vxEntry;
  private GenericEntry vyEntry;
  private GenericEntry radEntry;
  private GenericEntry moduleToggleEntry;
  /** initialize telemetry */
  private void initializeTelemetry() {
    ShuffleboardTab teleTab = Shuffleboard.getTab("Telemetry");
    ShuffleboardLayout driveLayout =
        teleTab.getLayout("drive", BuiltInLayouts.kList).withSize(1, 3);
    vxEntry = driveLayout.add("vx", vx).getEntry();
    vyEntry = driveLayout.add("vy", vy).getEntry();
    radEntry = driveLayout.add("rad", rad).getEntry();
    moduleToggleEntry = driveLayout.add("module toggle", toggleModule).getEntry();
  }

  /** update telemetry */
  private void updateTelemetry() {
    vxEntry.setDouble(vx);
    vyEntry.setDouble(vy);
    radEntry.setDouble(rad);
    moduleToggleEntry.setDouble(toggleModule);
  }

  /**
   * set field relative
   *
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
