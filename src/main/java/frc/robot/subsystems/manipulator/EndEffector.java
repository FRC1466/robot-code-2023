package frc.robot.subsystems.manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class EndEffector extends SubsystemBase {
  private final CANSparkMax gripperMotor;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;

  /** Create a new Gripper subsystem. */
  public EndEffector() {
    gripperMotor = new CANSparkMax(Intake.motorID, MotorType.kBrushless);
    gripperMotor.enableVoltageCompensation(12.0);
    gripperMotor.setSmartCurrentLimit(25);
    gripperMotor.burnFlash();
  }

  public void setRollers(double outputVolts) {
    gripperMotor.setVoltage(outputVolts);
  }

  public Command stop() {
    return runOnce(() -> gripperMotor.setVoltage(0));
  }

  public Command intake() {
    Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kBoth);
    System.out.println("intake scheduled");
    return runOnce(() -> debounce.calculate(false))
        .andThen(run(() -> setRollers(Intake.intakeV)))
        .until(() -> debounce.calculate(getFilteredCurrent() > Intake.stallCurrent))
        .finallyDo(
            (interrupted) -> {
              System.out.println("Current Limited.");
              setRollers(Intake.holdV);
            });
  }

  public Command drop() {
    return runOnce(() -> setRollers(Intake.dropV)).andThen(waitSeconds(1.0)).andThen(stop());
  }

  public Command launch() {
    return runOnce(() -> setRollers(Intake.launchV)).andThen(waitSeconds(1.0)).andThen(stop());
  }

  public Command powerLaunch() {
    return runOnce(() -> setRollers(Intake.powerLaunchV)).andThen(waitSeconds(0.5)).andThen(stop());
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public double getCurrent() {
    return gripperMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    filteredCurrent = currentFilter.calculate(getCurrent());
  }
}
