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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive2.auto.GoToScoring;
import frc.robot.commands.swervedrive2.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive2.auto.PathBuilder;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.VirtualFourBar;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The robot's subsystems
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VirtualFourBar m_arm = new VirtualFourBar();
  // private final LED m_led = new LED();

  private final PathBuilder m_builder = new PathBuilder(drivebase);

  private final CommandJoystick m_driverController = new CommandJoystick(OIConstants.driverID);
  private final CommandJoystick m_scoreController = new CommandJoystick(OIConstants.intakeID);

  // the default commands
  private final TeleopDrive closedFieldRel;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    closedFieldRel =
        new TeleopDrive(
            drivebase,
            () ->
                (Math.abs(m_driverController.getY()) > OIConstants.InputLimits.vyDeadband)
                    ? m_driverController.getY()
                    : 0,
            () ->
                (Math.abs(m_driverController.getX()) > OIConstants.InputLimits.vxDeadband)
                    ? m_driverController.getX()
                    : 0,
            () ->
                (Math.abs(m_driverController.getZ()) > OIConstants.InputLimits.radDeadband)
                    ? m_driverController.getZ()
                    : 0,
            () -> true,
            false);
    // Configure the button bindings
    configureButtonBindings();
    initializeChooser();

    drivebase.setDefaultCommand(closedFieldRel);
    m_arm.setDefaultCommand(
        new RunCommand(
            () ->
                m_arm.setArm(
                    MathUtil.clamp(
                        (m_driverController.getRawAxis(3) / 2.25) + 0.50, 0.15, 0.86)), // 0.62 1.42
            m_arm));
    // m_led.setDefaultCommand(Commands.run(() -> m_led.setColor(), m_led));
  }

  private void initializeChooser() {
    m_chooser.setDefaultOption(
        "Default Test",
        m_builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "Test Path",
                new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS))));

    m_chooser.addOption(
        "3 Score T1",
        m_builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "3 Score T1",
                new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS))));

    m_chooser.addOption(
        "1 Score + Dock T2",
        m_builder
            .getSwerveCommand(
                PathPlanner.loadPathGroup(
                    "1 Score + Dock T2",
                    new PathConstraints(
                        AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS)))
            .andThen(
                Commands.run(
                        () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                        drivebase)
                    .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0)));

    SmartDashboard.putData("CHOOSE", m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.button(4).onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController
        .button(3)
        .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d()), drivebase));
    m_scoreController.button(7).whileTrue(new GoToScoring(drivebase, POSITION.RIGHT));
    m_scoreController.button(8).whileTrue(new GoToScoring(drivebase, POSITION.MIDDLE));
    m_scoreController.button(9).whileTrue(new GoToScoring(drivebase, POSITION.LEFT));
    m_driverController
        .button(11)
        .whileTrue(
            Commands.run(
                    () -> drivebase.drive(drivebase.getBalanceTranslation(), 0, false, false),
                    drivebase)
                .until(() -> Math.abs(drivebase.getPlaneInclination().getDegrees()) < 2.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    return m_chooser.getSelected();
  }
}
