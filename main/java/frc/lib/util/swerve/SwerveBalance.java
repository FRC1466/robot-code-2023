package frc.lib.util.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveBalance {
    private PIDController balanceController;

    /**
     * Initialize SwerveBalance class.
     */
    public SwerveBalance() {
        balanceController = new PIDController(2.0, 0, 0);
    }

    /**
     * Update ChassisSpeeds for balancing based on pitch and roll. Uses a rough gradient descent through the
     * z function found in the last row of the rotation matrix calculatior for the transformation of a point.
     * @param pitch The pitch measurement of the gyro.
     * @param roll The roll measurement of the gyro.
     * @return ChassisSpeeds object to set to module states.
     */
    public ChassisSpeeds update(Rotation2d pitch, Rotation2d roll) {
        var xGrad = -pitch.getSin();
        var yGrad = pitch.getCos()*roll.getSin();

        var setpoint = Math.sqrt(xGrad*xGrad+yGrad*yGrad);
        var pidOutput = balanceController.calculate(setpoint, 0);

        var vxMetersPerSecond = xGrad * pidOutput;
        var vyMetersPerSecond = yGrad * pidOutput;
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, 0);
    }
}
