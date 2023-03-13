package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private CANSparkMax gripperMotor;

  public enum INTAKE {
    IN,
    OUT,
    STOP
  }

  private double currentGripper = 0;

  private INTAKE currentIntake;

  /** Create a new Gripper subsystem. */
  public Gripper() {

    gripperMotor = new CANSparkMax(GripperConstants.gripperID, MotorType.kBrushless);
    gripperMotor.enableVoltageCompensation(12.0);
    gripperMotor.setSmartCurrentLimit(30);
    gripperMotor.burnFlash();
  }

  public void ambientGripper() {
    gripperMotor.set(currentGripper);
  }

  public void setGripper(INTAKE intake) {
    currentIntake = intake;
    SmartDashboard.putString("Gripper", intake.toString());
    switch (intake) {
      case IN:
        currentGripper = GripperConstants.percentIn;
        break;
      case OUT:
        currentGripper = GripperConstants.percentOut;
        break;
      case STOP:
        currentGripper = 0;
        break;

      default:
        throw new IllegalArgumentException("Invalid intake enum position.");
    }
  }

  public INTAKE getCurrentIntake() {
    return currentIntake;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Gripper Encoder Position", encoder.getPosition());
    // SmartDashboard.putNumber("Gripper Amps", gripperMotor.getOutputCurrent());
  }
}
