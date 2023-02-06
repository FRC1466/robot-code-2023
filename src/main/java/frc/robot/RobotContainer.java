// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.ComplexAuto;
import frc.robot.commands.autonomous.GoToScoring;
import frc.robot.commands.autonomous.GoToScoring.POSITION;
import frc.robot.commands.autonomous.PathBuilder;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.constants.RobotConstants.OIConstants;
import frc.robot.subsystems.AdjustableTelemetry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VirtualFourBar;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final boolean isFieldRelative = true;
  private final AdjustableTelemetry m_tele = new AdjustableTelemetry();

  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final VirtualFourBar m_arm = new VirtualFourBar();

  private final PathBuilder m_builder = new PathBuilder(m_drive);

  private final Joystick m_driverController = new Joystick(OIConstants.driverID);
  private final Joystick m_scoreController = new Joystick(OIConstants.intakeID);

  // the default commands
  private final DriveCommand m_DriveCommand =
      new DriveCommand(m_drive, m_tele, m_driverController, isFieldRelative);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    initializeChooser();

    m_drive.setDefaultCommand(m_DriveCommand);
    m_arm.setDefaultCommand(
        new RunCommand(
            () ->
                m_arm.setArm(
                    MathUtil.clamp(
                        (m_driverController.getRawAxis(3) / 2.25) + 0.50, 0.15, 0.86)), // 0.62 1.42
            m_arm));
  }

  private void initializeChooser() {
    m_chooser.setDefaultOption("auto 1", new ComplexAuto(m_drive, m_tele, m_builder));

    m_chooser.addOption(
        "3 Score T1",
        m_builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "3 Score T1",
                new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS))));

    m_chooser.addOption(
        "2 Score + Dock T1",
        m_builder.getSwerveCommand(
            PathPlanner.loadPathGroup(
                "2 Score + Dock T1",
                new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS))));

    SmartDashboard.putData("CHOOSE", m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 4).onTrue(new InstantCommand(() -> m_drive.resetGyro()));
    new JoystickButton(m_driverController, 3)
        .onTrue(new InstantCommand(() -> m_drive.resetPose(new Pose2d())));
    new JoystickButton(m_scoreController, 7).onTrue(new GoToScoring(m_drive, POSITION.RIGHT));
    new JoystickButton(m_scoreController, 8).onTrue(new GoToScoring(m_drive, POSITION.MIDDLE));
    new JoystickButton(m_scoreController, 9).onTrue(new GoToScoring(m_drive, POSITION.LEFT));
    new JoystickButton(m_driverController, 12)
        .whileTrue(new RunCommand(() -> m_drive.driveAutoBalancingFull(), m_drive).until(() -> Math.abs(m_drive.getGyroPlaneInclination().getDegrees()) < 2.0));
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
