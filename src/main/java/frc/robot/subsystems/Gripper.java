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

  /** Create a new Gripper subsystem. */
  public Gripper() {
    gripperMotor = new CANSparkMax(34, MotorType.kBrushless);
    pidController = gripperMotor.getPIDController();
    initializePID();
    encoder = gripperMotor.getEncoder();
    encoder.setPosition(0);
  }

  public void setGripperMotorPercent(double a) {
    gripperMotor.set(a);
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

  public void setPID(double rotations) {
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gripper enc position", encoder.getPosition());
    pidController.setReference(-20.7, CANSparkMax.ControlType.kPosition);
  } // 0, -11.38, -20.7
}
