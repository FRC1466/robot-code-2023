// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;
  private final boolean isOpenLoop;
  private final SwerveController controller;
  private double lastTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(
      SwerveSubsystem swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier omega,
      BooleanSupplier driveMode,
      boolean isOpenLoop) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.isOpenLoop = isOpenLoop;
    this.controller = swerve.getSwerveController();
    this.lastTime = Timer.getFPGATimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3) * controller.config.maxSpeed;
    double yVelocity = Math.pow(vY.getAsDouble(), 3) * controller.config.maxSpeed;
    double angVelocity = Math.pow(omega.getAsDouble(), 3) * controller.config.maxAngularVelocity;
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    if (Math.abs(xVelocity) > 0 || Math.abs(yVelocity) > 0 || Math.abs(angVelocity) > 0) {
      lastTime = Timer.getFPGATimestamp();
      swerve.drive(
          new Translation2d(xVelocity, yVelocity),
          angVelocity,
          driveMode.getAsBoolean(),
          isOpenLoop);
    } else if (Timer.getFPGATimestamp() - lastTime > Constants.STOP_SECONDS) {
      swerve.lockPose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
