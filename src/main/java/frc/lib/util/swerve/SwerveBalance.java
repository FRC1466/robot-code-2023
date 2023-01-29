package frc.lib.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveBalance {
    private double scale;

    /**
     * Initialize SwerveBalance class.
     * @param scale the scale to apply the gradients by
     */
    public SwerveBalance(double scale) {
        this.scale = scale;
    }

    /**
     * Update ChassisSpeeds for balancing based on pitch and roll. Uses a rough gradient descent through the
     * z function found in the last row of the rotation matrix calculatior for the transformation of a point.
     * @param pitch The pitch measurement of the gyro.
     * @param roll The roll measurement of the gyro.
     * @return ChassisSpeeds object to set to module states.
     */
    public ChassisSpeeds update(Rotation2d pitch, Rotation2d roll) {
        var xGrad = -pitch.getTan(); // sin/cos pitch
        var yGrad = pitch.getCos()*roll.getTan(); // sin/cos roll

        var vxMetersPerSecond = xGrad * scale;
        var vyMetersPerSecond = yGrad * scale;
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, 0);
    }
}
