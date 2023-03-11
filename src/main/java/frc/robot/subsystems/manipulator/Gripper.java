package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private CANSparkMax gripperMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  public enum INTAKE {
    OPEN,
    CUBE,
    CONE,
    STORE
  }

  private double currentGripper = GripperConstants.positionStore;

  private INTAKE currentIntake;

  /** Create a new Gripper subsystem. */
  public Gripper() {

    // gripperMotor = new CANSparkMax(GripperConstants.gripperID, MotorType.kBrushless);
    // gripperMotor.enableVoltageCompensation(12.0);
    // gripperMotor.setSmartCurrentLimit(40);
    // gripperMotor.burnFlash();

    // pidController = gripperMotor.getPIDController();
    // initializePID();

    // encoder = gripperMotor.getEncoder();
    // initializeEncoder();
  }

  private void initializePID() {
    pidController.setP(GripperConstants.gripperPosition.P);
    pidController.setI(GripperConstants.gripperPosition.I);
    pidController.setD(GripperConstants.gripperPosition.D);
    pidController.setIZone(GripperConstants.gripperPosition.integralZone);
    pidController.setFF(GripperConstants.gripperPosition.F);
    pidController.setOutputRange(
        -GripperConstants.gripperPosition.peakOutput, GripperConstants.gripperPosition.peakOutput);
  }

  private void initializeEncoder() {
    encoder.setPosition(0);
  }

  public void ambientGripper() {
    // pidController.setReference(currentGripper, CANSparkMax.ControlType.kPosition);
  }

  public void setGripper(INTAKE intake) {
    currentIntake = intake;
    SmartDashboard.putString("Gripper", intake.toString());
    switch (intake) {
      case OPEN:
        currentGripper = GripperConstants.positionOpen;
        break;
      case CUBE:
        currentGripper = GripperConstants.cubeInConeOut;
        break;
      case CONE:
        currentGripper = GripperConstants.cubeOutConeIn;
        break;
      case STORE:
        currentGripper = GripperConstants.positionStore;
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
