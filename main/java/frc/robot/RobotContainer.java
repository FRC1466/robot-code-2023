// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AdjustableTelemetry;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.ComplexAuto;
import frc.robot.commands.autonomous.GoToPose;
import frc.robot.commands.autonomous.GoToScoring;
import frc.robot.commands.autonomous.PathBuilder;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.constants.RobotConstants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final PathBuilder m_builder = new PathBuilder(m_drive);

  private final Joystick m_driverController = new Joystick(OIConstants.driverID);



  // the default commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_drive, m_tele, m_driverController, isFieldRelative);

  private final GoToPose goToPose = new GoToPose(
    new Pose2d(2.18, 1.74, new Rotation2d()), new Rotation2d(), 
    new PathConstraints(2, 2), m_drive);
  private final GoToScoring goToScoring = new GoToScoring(m_drive);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    Command auto1 = new ComplexAuto(m_drive, m_tele, m_builder);
    Command auto2 = m_builder.getSwerveCommand(PathPlanner.loadPathGroup("3 Score T1", 
    new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS)));
    m_chooser.setDefaultOption("auto 1", auto1);
    m_chooser.addOption("auto 2", auto2);
    SmartDashboard.putData("CHOOSE", m_chooser);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drive.setDefaultCommand(
        m_DriveCommand
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 4).onTrue(new InstantCommand(() -> m_drive.resetGyro()));
    new JoystickButton(m_driverController, 3).onTrue(new InstantCommand(() -> m_drive.resetPose(new Pose2d())));
    // new JoystickButton(m_driverController, 7).whileTrue(goToScoring.getCommand(1, m_drive.getPose()));
    // new JoystickButton(m_driverController, 9).whileTrue(goToScoring.getCommand(2, m_drive.getPose()));
    // new JoystickButton(m_driverController, 11).whileTrue(goToScoring.getCommand(3, m_drive.getPose()));
    // new JoystickButton(m_driverController, 12).whileTrue(new RepeatCommand(new InstantCommand(() -> m_drive.driveAutoBalancingFull())));
  }

  /**
   * Use this to pass the  autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAuto() {
    return m_chooser.getSelected();
  }

}
