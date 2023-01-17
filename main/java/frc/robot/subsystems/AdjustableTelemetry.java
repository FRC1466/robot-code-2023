package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.Swerve;

public class AdjustableTelemetry {
    private ShuffleboardTab tuningTab;

    private GenericEntry posPEntry;
    private GenericEntry posIEntry;
    private GenericEntry posDEntry;
    private GenericEntry posFEntry;
    private GenericEntry posIzoneEntry;
    private GenericEntry velPEntry;
    private GenericEntry velIEntry;
    private GenericEntry velDEntry;
    private GenericEntry velFEntry;
    private GenericEntry velIzoneEntry;
    private GenericEntry translationPEntry;
    private GenericEntry translationIEntry;
    private GenericEntry translationDEntry;
    private GenericEntry thetaPEntry;
    private GenericEntry thetaIEntry;
    private GenericEntry thetaDEntry;

    private GenericEntry vxLimit;
    private GenericEntry vyLimit;
    private GenericEntry rotLimit;

    /**
     * initializes the adjustable telemetry
     */
    public AdjustableTelemetry() {
        tuningTab = Shuffleboard.getTab("Tuning");

        initializePIDUpdate();
        initializeDriveLimits();
    }
    
    private void initializePIDUpdate() {

        ShuffleboardLayout positionPIDLayout = tuningTab
            .getLayout("Position PID", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(1, 5);

        posPEntry = positionPIDLayout.add("P", Swerve.driveGainsPosition.P).getEntry();
        posIEntry = positionPIDLayout.add("I", Swerve.driveGainsPosition.I).getEntry();
        posDEntry = positionPIDLayout.add("D", Swerve.driveGainsPosition.D).getEntry();
        posFEntry = positionPIDLayout.add("F", Swerve.driveGainsPosition.F).getEntry();
        posIzoneEntry = positionPIDLayout.add("Izone", Swerve.driveGainsPosition.integralZone).getEntry();

        ShuffleboardLayout velocityPIDLayout = tuningTab
            .getLayout("Velocity PID", BuiltInLayouts.kList)
            .withPosition(1, 0)
            .withSize(1, 5);

        velPEntry = velocityPIDLayout.add("P", Swerve.driveGainsVelocity.P).getEntry();
        velIEntry = velocityPIDLayout.add("I", Swerve.driveGainsVelocity.I).getEntry();
        velDEntry = velocityPIDLayout.add("D", Swerve.driveGainsVelocity.D).getEntry();
        velFEntry = velocityPIDLayout.add("F", Swerve.driveGainsVelocity.F).getEntry();
        velIzoneEntry = velocityPIDLayout.add("Izone", Swerve.driveGainsVelocity.integralZone).getEntry();

        ShuffleboardLayout autoPIDLayout = tuningTab
            .getLayout("Auto PID", BuiltInLayouts.kList)
            .withSize(1, 6);

        translationPEntry = autoPIDLayout.add("P_t", AutoConstants.translationController.P).getEntry();
        translationIEntry = autoPIDLayout.add("I_t", AutoConstants.translationController.I).getEntry();
        translationDEntry = autoPIDLayout.add("D_t", AutoConstants.translationController.D).getEntry();
        thetaPEntry = autoPIDLayout.add("P_r", AutoConstants.thetaController.P).getEntry();
        thetaIEntry = autoPIDLayout.add("I_r", AutoConstants.thetaController.I).getEntry();
        thetaDEntry = autoPIDLayout.add("D_r", AutoConstants.thetaController.D).getEntry();
    } 

    private void initializeDriveLimits() {
        ShuffleboardLayout driveLimits = tuningTab
            .getLayout("Drive Limits", BuiltInLayouts.kList)
            .withSize(1, 4);
        
        vxLimit = driveLimits.add("vx", Swerve.Limits.vx).getEntry();
        vyLimit = driveLimits.add("vy", Swerve.Limits.vy).getEntry();
        rotLimit = driveLimits.add("rad", Swerve.Limits.rad).getEntry();
    }

    public void updatePIDConstants() {
        Swerve.driveGainsPosition.P = posPEntry.getDouble(0);
        Swerve.driveGainsPosition.I = posIEntry.getDouble(0);
        Swerve.driveGainsPosition.D = posDEntry.getDouble(0);
        Swerve.driveGainsPosition.F = posFEntry.getDouble(0);
        Swerve.driveGainsPosition.integralZone = posIzoneEntry.getDouble(0);
    
        Swerve.driveGainsVelocity.P = velPEntry.getDouble(0);
        Swerve.driveGainsVelocity.I = velIEntry.getDouble(0);
        Swerve.driveGainsVelocity.D = velDEntry.getDouble(0);
        Swerve.driveGainsVelocity.F = velFEntry.getDouble(0);
        Swerve.driveGainsVelocity.integralZone = velIzoneEntry.getDouble(0);

        AutoConstants.translationController.P = translationPEntry.getDouble(0);
        AutoConstants.translationController.I = translationIEntry.getDouble(0);
        AutoConstants.translationController.D = translationDEntry.getDouble(0);

        AutoConstants.thetaController.P = thetaPEntry.getDouble(0);
        AutoConstants.thetaController.I = thetaIEntry.getDouble(0);
        AutoConstants.thetaController.D = thetaDEntry.getDouble(0);
    }

    public void updateDriveLimits() {
        Swerve.Limits.vx = vxLimit.getDouble(Swerve.Limits.vx);
        Swerve.Limits.vy = vyLimit.getDouble(Swerve.Limits.vy);
        Swerve.Limits.rad = rotLimit.getDouble(Swerve.Limits.rad);
    }

}
