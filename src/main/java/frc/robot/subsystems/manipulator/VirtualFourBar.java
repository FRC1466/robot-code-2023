package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import webblib.math.ArmPIDController;

public class VirtualFourBar extends SubsystemBase {
  private WPI_TalonFX armMotor;
  private DutyCycleEncoder absoluteArmEncoder;
  private ArmPIDController armPID;
  private VirtualFourBarSimulation sim;
  private Rotation2d localSetpoint;
  private DoubleSupplier overrideFeedforward = () -> 0.0;
  private boolean disabled = false;

  public enum ARM {
    GROUND,
    VERTICAL,
    STORAGE,
    STATION,
    MID
  }

  private Rotation2d currentArm = Rotation2d.fromRadians(ArmConstants.minRadians);

  /** Create a new VirtualFourBar subsystem. */
  public VirtualFourBar() {
    armMotor = new WPI_TalonFX(ArmConstants.armPort);
    configArmMotor();

    absoluteArmEncoder = new DutyCycleEncoder(ArmConstants.dutyCyclePort);
    absoluteArmEncoder.setDutyCycleRange(0, 1);
    absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new ArmPIDController(
            ArmConstants.armPosition.P, ArmConstants.armPosition.I, ArmConstants.armPosition.D);
    armPID.setAvoidanceRange(
        Rotation2d.fromRadians(ArmConstants.maxRadians),
        Rotation2d.fromRadians(ArmConstants.minRadians));
    armPID.setTolerance(0.15);

    if (Robot.isSimulation()) {
      sim = new VirtualFourBarSimulation(absoluteArmEncoder);
      SmartDashboard.putData("Arm Sim", sim.getMech2d());
    }

    setGoal(Rotation2d.fromRadians(ArmConstants.minRadians));
    setDefaultCommand(hold());
  }

  @Override
  public void simulationPeriodic() {
    sim.update(armMotor.get());
  }

  /** Configure arm motor. */
  private void configArmMotor() {
    armMotor.configFactoryDefault();
    armMotor.configAllSettings(ArmConstants.ArmConfig.motorConfig);
  }

  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition =
        absoluteArmEncoder.getAbsolutePosition() / ArmConstants.dutyCycleResolution;
    return Rotation2d.fromRotations(initialPosition)
        .minus(Rotation2d.fromRotations(ArmConstants.absolutePositionOffset));
  }

  /**
   * Gets position of arm in radians. Assumes the arm being level within frame is the 0 point on the
   * x axis. Assumes CCW+.
   *
   * @return position in rad.
   */
  public Rotation2d getPosition() {
    return ArmConstants.encoderInverted
        ? getShiftedAbsoluteDistance().unaryMinus()
        : getShiftedAbsoluteDistance();
  }

  /**
   * Set arm with PID.
   *
   * @param setpoint setpoint in radians.
   */
  public void setGoal(Rotation2d setpoint) {
    localSetpoint = setpoint;
    // armPID.setSetpoint(setpoint);
    SmartDashboard.putNumber("Arm PID Setpoint", setpoint.getRadians());
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            armPID.calculate(getPosition(), localSetpoint),
            -ArmConstants.armPosition.peakOutput,
            ArmConstants.armPosition.peakOutput);
    var feedforward = getPosition().getCos() * ArmConstants.gravityFF;

    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());

    SmartDashboard.putNumber("Arm PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setMotor(double percent) {
    armMotor.set(percent);
  }

  public void setFeedforward(DoubleSupplier ff) {
    System.out.println("ff: " + ff.toString());
    overrideFeedforward = ff;
  }

  public Command ground() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.maxRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command loft() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.loftRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command station() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.stationDegrees)))
        .andThen(holdUntilSetpoint());
  }

  public Command mid() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.midDegrees)))
        .andThen(holdUntilSetpoint());
  }

  public Command midScore() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.midDegreesScore)))
        .andThen(holdUntilSetpoint());
  }

  public Command high() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.highDegrees)))
        .andThen(holdUntilSetpoint());
  }

  public Command storeLaunchReady() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.launchRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command highLaunchReady() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.highLaunchDegrees)))
        .andThen(holdUntilSetpoint());
  }

  public Command store() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(ArmConstants.minRadians)))
        .andThen(holdUntilSetpoint());
  }

  public Command vertical() {
    return runOnce(() -> setGoal(Rotation2d.fromDegrees(ArmConstants.verticalDegrees)))
        .andThen(holdUntilSetpoint());
  }

  public Command hold() {
    return Commands.run(this::setArmHold, this);
  }

  public Command holdUntilSetpoint() {
    return hold()
        .raceWith(Commands.waitSeconds(0.3).andThen(Commands.waitUntil(this::isAtSetpoint)));
  }

  public CommandBase toggleDisable() {
    return runOnce(
        () -> {
          disabled = !disabled;
        });
  }

  /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  public boolean isAtSetpoint() {
    SmartDashboard.putBoolean("Arm PID at setpoint", armPID.atSetpoint());
    return armPID.atSetpoint();
  }

  public Supplier<Translation3d> getCOM() {
    var rest = new Translation2d(Constants.ARM_LENGTH / 2, 0);
    var rotatedRest = rest.rotateBy(getPosition());
    return () ->
        Constants.INITIAL_ARM_MOUNT.plus(
            new Translation3d(-rotatedRest.getX(), 0, rotatedRest.getY()));
  }

  @Override
  public void periodic() {
    setArmHold();

    SmartDashboard.putData(absoluteArmEncoder);
    SmartDashboard.putNumber("Arm Raw Absolute Encoder", absoluteArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Processed Absolute Encoder", getPosition().getRadians());
    SmartDashboard.putNumber("Arm PID error", armPID.getPositionError());
    SmartDashboard.putString("Arm COM", getCOM().get().toString());
    SmartDashboard.putBoolean("Arm Disabled", disabled);
  }
}
