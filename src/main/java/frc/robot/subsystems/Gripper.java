package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    CONE
  }

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

  public void setGripper(INTAKE intake) {
    switch (intake) {
      case OPEN:
        pidController.setReference(
            GripperConstants.positionOpen, CANSparkMax.ControlType.kPosition);
        break;
      case CUBE:
        pidController.setReference(
            GripperConstants.positionCube, CANSparkMax.ControlType.kPosition);
        break;
      case CONE:
        pidController.setReference(
            GripperConstants.positionCone, CANSparkMax.ControlType.kPosition);
        break;
      default:
        throw new IllegalArgumentException("Invalid intake enum position.");
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gripper SPARKMAX encoder position", encoder.getPosition());
  }
}
