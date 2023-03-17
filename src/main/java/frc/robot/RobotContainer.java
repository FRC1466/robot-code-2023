// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.InputLimits;
import frc.robot.commands.AutoMap;
import frc.robot.commands.Superstructure;
import frc.robot.commands.swervedrive.auto.GoToScoring;
import frc.robot.commands.swervedrive.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive.auto.PathBuilder;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.PDH;
import frc.robot.subsystems.manipulator.EndEffector;
import frc.robot.subsystems.manipulator.VirtualFourBar;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  // The robot's subsystems
  private final VirtualFourBar arm = new VirtualFourBar();
  private final EndEffector effector = new EndEffector();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), arm.getCOM());
  // private final LED m_led = new LED();
  private final PDH pdh = new PDH();
  private final Superstructure superstructure = new Superstructure(effector, arm);
  private final AutoMap autoMap = new AutoMap(superstructure, effector, arm);
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getEventMap());

  private final CommandJoystick driverController = new CommandJoystick(OIConstants.driverID);
  private final CommandJoystick scoreController = new CommandJoystick(OIConstants.intakeID);

  // the default commands
  private final TeleopDrive closedFieldRel =
      new TeleopDrive(
          drivebase,
          () -> MathUtil.applyDeadband(-driverController.getY(), InputLimits.vxDeadband),
          () -> MathUtil.applyDeadband(-driverController.getX(), InputLimits.vyDeadband),
          () ->
              MathUtil.applyDeadband(
                  -driverController.getZ() * InputLimits.defaultAngScale, InputLimits.angDeadband),
          driverController.button(8).negate(),
          false, arm.getCOM());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    initializeChooser();
  }

  private void initializeChooser() {

    chooser.addOption(
        "Default Test",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "Test Path", new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    // chooser.setDefaultOption(
    //     "Default Test Full",
    //     builder.getSwerveCommand(
    //         PathPlanner.loadPathGroup(
    //             "Test Full Path",
    //             new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    // chooser.addOption(
    //     "3 Score T1",
    //     builder.getSwerveCommand(
    //         PathPlanner.loadPathGroup(
    //             "3 Score T1", new PathConstraints(Auton.maxSpeedMPS,
    // Auton.maxAccelerationMPS))));

    chooser.addOption(
        "2 Score + Dock T1",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "2 Score + Dock T1",
                new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS)))
                .andThen(autoBalance()).andThen(Commands.waitSeconds(1.0)).andThen(autoBalance()));

    chooser.addOption(
        "1 Score + Dock T2",
        builder
            .getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "1 Score + Dock T2",
                    new PathConstraints(1.5, 2)))
            .andThen(autoBalance()).andThen(Commands.waitSeconds(1.0)).andThen(autoBalance()));

    SmartDashboard.putData("CHOOSE", chooser);
  }

  //
  // Brainstorming for controls:
  // 1) Pickup floor cube on press. when released, grab the cube and
  // store it
  // 2) Pickup floor cone on press. when released, grab the cone and store it
  // note: combine 1&2, 3&4, 5&6 w/ color sensor?
  // 3) Pickup station cube on press (NONALIGN). when released, grab the cube and store it
  // 4) Pickup station cone on press (NONALIGN). when released, grab the cube and store it (nonalign
  // option depends on how good align is & vice versa)
  // 5) Pickup station cube on press (ALIGN). when released, grab the cube and store it
  // 6) Pickup station cone on press (ALIGN). when released, grab the cube and store it
  // 7) manual low scoring button
  // 8) manual mid scoring button
  // Button box auto scoring
  // worst case for manipulator: 8, best case for manipulator: 2 (possibly 4 with scoring
  // redundancy)
  //
  // 1) Non-field relative toggle
  // 2) GYRO RESET
  // 3) Slower drive
  // 4) auto balance (could be helpful idk, not the most necessary)

  private Command autoBalance() {
    return Commands.run(
            () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false), drivebase)
        .until(
            () -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < Auton.balanceLimitDeg);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
    drivebase.setDefaultCommand(closedFieldRel);
    // m_led.setDefaultCommand(Commands.run(m_led::setColor, m_led));

    driverController.povDown().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverController.povUp().whileTrue(autoBalance());
    driverController
        .povRight()
        .whileTrue(Commands.runOnce(() -> drivebase.softVisionMeasurements = false))
        .whileFalse(Commands.runOnce(() -> drivebase.softVisionMeasurements = true));

    driverController
        .button(7)
        .whileTrue(
            new TeleopDrive(
                drivebase,
                () ->
                    MathUtil.applyDeadband(
                        -driverController.getY() * InputLimits.reduced, InputLimits.vxDeadband),
                () ->
                    MathUtil.applyDeadband(
                        -driverController.getX() * InputLimits.reduced, InputLimits.vyDeadband),
                () ->
                    MathUtil.applyDeadband(
                        -driverController.getZ() * InputLimits.reduced, InputLimits.angDeadband),
                driverController.button(8).negate(),
                false, arm.getCOM()));

    new Trigger(DriverStation::isTeleopEnabled).onTrue(superstructure.store());

    driverController.button(9).whileTrue(arm.mid()).whileFalse(superstructure.dropStore());
    driverController.button(10).whileTrue(arm.high()).whileFalse(superstructure.launchStore());

    driverController
        .button(2)
        .whileTrue(superstructure.pickupStation())
        .whileFalse(superstructure.store());

    driverController
        .button(3)
        .whileTrue(superstructure.pickupGround())
        .whileFalse(superstructure.store());

    driverController.button(4).whileTrue(arm.loft()).whileFalse(superstructure.launchStore());

    driverController.button(13).whileTrue(effector.intake()).whileFalse(effector.stop());
    driverController.button(12).whileTrue(effector.drop()).whileFalse(effector.stop());
    driverController.button(11).whileTrue(effector.launch()).whileFalse(effector.stop());
    driverController.button(14).whileTrue(arm.highLaunchReady())
            .whileFalse(superstructure.launchConeToHigh().andThen(arm.store()));

    scoreController
        .button(1)
        .whileTrue(new GoToScoring(drivebase, POSITION.RIGHT, -0.5).getCommand().alongWith(arm.ground()))
        .whileFalse(superstructure.dropStore());

    scoreController
        .button(2)
        .whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE, -0.5).getCommand().alongWith(arm.ground()))
        .whileFalse(superstructure.dropStore());

    scoreController
        .button(3)
        .whileTrue(new GoToScoring(drivebase, POSITION.LEFT, -0.5).getCommand().alongWith(arm.ground()))
        .whileFalse(superstructure.dropStore());

    scoreController
        .button(4)
        .whileTrue(new GoToScoring(drivebase, POSITION.RIGHT, 0.0).getCommand().alongWith(arm.mid()))
        .whileFalse(superstructure.dropStore());

    scoreController
        .button(5)
        .whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE, 0.0).getCommand().alongWith(arm.mid()))
        .whileFalse(superstructure.dropStore());

    scoreController
        .button(6)
        .whileTrue(new GoToScoring(drivebase, POSITION.LEFT, 0.0).getCommand().alongWith(arm.mid()))
        .whileFalse(superstructure.dropStore());
    scoreController
            .button(7)
            .whileTrue(new GoToScoring(drivebase, POSITION.RIGHT, 0.0).getCommand().alongWith(arm.highLaunchReady()))
            .whileFalse(superstructure.launchConeToHigh().andThen(arm.store()));
    scoreController
        .button(8)
        .whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE, 0.0).getCommand().alongWith(arm.high()))
        .whileFalse(superstructure.launchStore());
    scoreController
            .button(9)
            .whileTrue(new GoToScoring(drivebase, POSITION.LEFT, 0.0).getCommand().alongWith(arm.highLaunchReady()))
            .whileFalse(superstructure.launchConeToHigh().andThen(arm.store()));

    new Trigger(drivebase::isMoving)
        .debounce(10, Debouncer.DebounceType.kBoth)
        .onTrue(pdh.switchableOn())
        .onFalse(pdh.switchableOff());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    return chooser.getSelected();
  }
}
