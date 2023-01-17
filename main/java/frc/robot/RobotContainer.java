// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AdjustableTelemetry;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Autonomous.ComplexAuto;
import frc.robot.constants.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final boolean isFieldRelative = true;
  private final AdjustableTelemetry m_tele = new AdjustableTelemetry();

  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();

  private final Joystick m_driverController = new Joystick(OIConstants.driverID);



  // the default commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_drive, m_tele, m_driverController, isFieldRelative);

  




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    

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
    new JoystickButton(m_driverController, 3).onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));
  }
    



  /**
   * Use this to pass the  autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAuto() {
    return new ComplexAuto(m_drive, m_tele);
  }



}
