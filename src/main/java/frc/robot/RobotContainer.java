// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive2.auto.GoToScoring;
import frc.robot.commands.swervedrive2.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive2.auto.PathBuilder;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PDH;
import frc.robot.subsystems.VirtualFourBar;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.io.File;

import javax.swing.GrayFilter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  // The robot's subsystems
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VirtualFourBar arm = new VirtualFourBar();
  private final Gripper gripper = new Gripper();
  // private final LED m_led = new LED();
  private final PDH pdh = new PDH();

  private final PathBuilder builder = new PathBuilder(drivebase);

  private final CommandJoystick driverController = new CommandJoystick(OIConstants.driverID);
  private final CommandJoystick scoreController = new CommandJoystick(OIConstants.intakeID);

  // the default commands
  private final TeleopDrive closedFieldRel;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    closedFieldRel =
        new TeleopDrive(
            drivebase,
            () ->
                (Math.abs(driverController.getY()) > OIConstants.InputLimits.vyDeadband)
                    ? -driverController.getY()
                    : 0,
            () ->
                (Math.abs(driverController.getX()) > OIConstants.InputLimits.vxDeadband)
                    ? -driverController.getX()
                    : 0,
            () ->
                (Math.abs(driverController.getZ()) > OIConstants.InputLimits.radDeadband)
                    ? -driverController.getZ()
                    : 0,
            driverController.button(3),
            false);
    // Configure the button bindings
    configureButtonBindings();
    initializeChooser();

    drivebase.setDefaultCommand(closedFieldRel);
    arm.setDefaultCommand(
        new RunCommand(
            () ->
                arm.setArm(MathUtil.clamp(driverController.getRawAxis(3) / 2 * Math.PI + (Math.PI/2), -0.68, 4.34)),
            arm)); // 0.68
    // arm.setDefaultCommand(Commands.run(()-> arm.setArmPercent(scoreController.getRawAxis(1)/3),
    // arm));
    // m_led.setDefaultCommand(Commands.run(() -> m_led.setColor(), m_led));
  }

  private void initializeChooser() {

    chooser.setDefaultOption(
        "Default Test",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "Test Path", new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    chooser.addOption(
        "3 Score T1",
        builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "3 Score T1", new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));

    chooser.addOption(
        "1 Score + Dock T2",
        builder
            .getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "1 Score + Dock T2",
                    new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS)))
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0)));

    SmartDashboard.putData("CHOOSE", chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.button(4).onTrue(new InstantCommand(drivebase::zeroGyro));
    driverController
        .button(3)
        .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d()), drivebase));
    scoreController.button(1).whileTrue(new GoToScoring(drivebase, POSITION.RIGHT));
    scoreController.button(2).whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE));
    scoreController.button(3).whileTrue(new GoToScoring(drivebase, POSITION.LEFT));
    driverController
        .button(5)
        .whileTrue(
            Commands.run(
                    () -> drivebase.drive(drivebase.getBalanceTranslation().times(-1), 0, false, false),
                    drivebase)
                .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0));
    new Trigger(drivebase::isMoving).onTrue(Commands.runOnce(() -> pdh.setSwitchableChannel(true), pdh));
    new Trigger(drivebase::isMoving).onFalse(Commands.runOnce(() -> pdh.setSwitchableChannel(false), pdh));
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
