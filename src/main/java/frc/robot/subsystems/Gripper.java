package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.GripperConstants;

public class Gripper extends SubsystemBase {
  private CANSparkMax gripperMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  public Gripper() {
    gripperMotor = new CANSparkMax(24, MotorType.kBrushless);
    pidController = gripperMotor.getPIDController();
    initializePID();
    encoder = gripperMotor.getEncoder();
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
  }
}
