package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private final CANSparkMax gripperMotor;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;

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

    gripperMotor = new CANSparkMax(GripperConstants.gripperID, MotorType.kBrushless);

    pidController = gripperMotor.getPIDController();
    initializePID();

    encoder = gripperMotor.getEncoder();
    initializeEncoder();
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
    pidController.setReference(currentGripper, CANSparkMax.ControlType.kPosition);
  }

  public void setGripper(INTAKE intake) {
    currentIntake = intake;
    switch (intake) {
      case OPEN:
        currentGripper = GripperConstants.positionOpen;
        break;
      case CUBE:
        currentGripper = GripperConstants.positionCube;
        break;
      case CONE:
        currentGripper = GripperConstants.positionCone;
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
    SmartDashboard.putNumber("Gripper SPARKMAX encoder position", encoder.getPosition());
  }
}
