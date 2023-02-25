// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive.AutoMap;
import frc.robot.commands.swervedrive.auto.GoToScoring;
import frc.robot.commands.swervedrive.auto.GoToScoring.POSITION;
import frc.robot.commands.swervedrive.auto.PathBuilder;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.PDH;
import frc.robot.subsystems.manipulator.Gripper;
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
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VirtualFourBar arm = new VirtualFourBar();
  private final Gripper gripper = new Gripper();
  // private final LED m_led = new LED();
  private final PDH pdh = new PDH();

  private final AutoMap autoMap = new AutoMap(gripper, arm);
  private final PathBuilder builder = new PathBuilder(drivebase, autoMap.getMap());

  private final CommandJoystick driverController = new CommandJoystick(OIConstants.driverID);
  private final CommandJoystick scoreController = new CommandJoystick(OIConstants.intakeID);

  // the default commands
  private final TeleopDrive closedFieldRel =
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
          () -> true, // driverController.button(3).negate(),
          false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    initializeChooser();
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
            .andThen(autoBalance()));

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
    arm.setDefaultCommand(Commands.run(arm::ambientArm, arm));
    // m_led.setDefaultCommand(Commands.run(m_led::setColor, m_led));
    gripper.setDefaultCommand(Commands.run(gripper::ambientGripper, gripper));

    driverController.povDown().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverController.povUp().whileTrue(autoBalance());

    driverController
        .button(7)
        .whileTrue(
            new TeleopDrive(
                drivebase,
                () ->
                    (Math.abs(driverController.getY()) > OIConstants.InputLimits.vyDeadband)
                        ? -driverController.getY() * OIConstants.InputLimits.reduced
                        : 0,
                () ->
                    (Math.abs(driverController.getX()) > OIConstants.InputLimits.vxDeadband)
                        ? -driverController.getX() * OIConstants.InputLimits.reduced
                        : 0,
                () ->
                    (Math.abs(driverController.getZ()) > OIConstants.InputLimits.radDeadband)
                        ? -driverController.getZ() * OIConstants.InputLimits.reduced
                        : 0,
                () -> true, // driverController.button(3).negate(),
                false));

    // new
    // Trigger(DriverStation::isTeleopEnabled).onTrue(autoMap.getCommandInMap(AutoMap.PickupCubeAndStore));

    driverController
        .button(2)
        .whileTrue(autoMap.getCommandInMap(AutoMap.ScoreArmMid))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    driverController
        .button(3)
        .whileTrue(autoMap.getCommandInMap(AutoMap.PickupGroundReady))
        .whileFalse(autoMap.getCommandInMap(AutoMap.PickupConeAndStore));

    driverController
        .button(4)
        .whileTrue(autoMap.getCommandInMap(AutoMap.PickupGroundReady))
        .whileFalse(autoMap.getCommandInMap(AutoMap.PickupCubeAndStore));

    driverController
        .button(5)
        .whileTrue(autoMap.getCommandInMap(AutoMap.PickupLoadingStationReady))
        .whileFalse(autoMap.getCommandInMap(AutoMap.PickupConeAndStore));

    driverController
        .button(6)
        .whileTrue(autoMap.getCommandInMap(AutoMap.PickupLoadingStationReady))
        .whileFalse(autoMap.getCommandInMap(AutoMap.PickupCubeAndStore));

    scoreController
        .button(1)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.RIGHT)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmLow)))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    scoreController
        .button(2)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.MIDDLE)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmLow)))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    scoreController
        .button(3)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.LEFT)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmLow)))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    scoreController
        .button(4)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.RIGHT)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmMid)))
        .onFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    scoreController
        .button(5)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.MIDDLE)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmMid)))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    scoreController
        .button(6)
        .whileTrue(
            new GoToScoring(drivebase, POSITION.LEFT)
                .getCommand()
                .alongWith(autoMap.getCommandInMap(AutoMap.ScoreArmMid)))
        .whileFalse(autoMap.getCommandInMap(AutoMap.DropObjectAndStore));

    // new Trigger(drivebase::isMoving)
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> pdh.setSwitchableChannel(true), () -> pdh.setSwitchableChannel(false),
    // pdh));
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
