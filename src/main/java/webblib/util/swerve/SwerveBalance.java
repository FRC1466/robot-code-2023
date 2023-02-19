package webblib.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import webblib.math.BetterMath;

public class SwerveBalance {
  private double scale;
  private double scalePow;

  /**
   * Initialize SwerveBalance class.
   *
   * @param scale The scalar to apply to the gradients.
   * @param scalePow Weight the result to be nonlinear (faster to balance when farther away). Set to
   *     1 to be linear. Must be greater than 0.
   */
  public SwerveBalance(double scale, double scalePow) {
    if (scalePow <= 0) {
      throw new IllegalArgumentException("scalePow must be greater than 0");
    }
    this.scale = scale;
    this.scalePow = scalePow;
  }

  /**
   * Update ChassisSpeeds for balancing based on pitch and roll. Create a plane through the z
   * function found in the last row of the rotation matrix calculatior for the transformation of a
   * point. Is essentially the derivative plane of a function in R3. Scales it according to
   * constants in instance.
   *
   * @param pitch The pitch measurement of the gyro.
   * @param roll The roll measurement of the gyro.
   * @return ChassisSpeeds object to set to module states.
   */
  public Translation2d update(Rotation2d pitch, Rotation2d roll) {
    var xGrad = pitch.getTan();
    var yGrad = -roll.getTan();

    var vyMetersPerSecond =
        BetterMath.signedAbsFunc(yGrad, (x) -> Math.pow(Math.abs(x * scale), scalePow));
    var vxMetersPerSecond =
        BetterMath.signedAbsFunc(xGrad, (x) -> Math.pow(Math.abs(x * scale), scalePow));

    return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
  }
}
