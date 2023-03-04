package swervelib.imu;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/** SwerveIMU interface for the Pigeon2 */
public class Pigeon2Swerve extends SwerveIMU {

  /** Pigeon2 IMU device. */
  WPI_Pigeon2 imu;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus) {
    imu = new WPI_Pigeon2(canid, canbus);
    Pigeon2Configuration config = new Pigeon2Configuration();
    imu.configAllSettings(config);
    SmartDashboard.putData(imu);
  }

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid) {
    this(canid, "");
  }

  /** Reset IMU to factory default. */
  @Override
  public void factoryDefault() {
    imu.configFactoryDefault();
  }

  /** Clear sticky faults on IMU. */
  @Override
  public void clearStickyFaults() {
    imu.clearStickyFaults();
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Angle in degrees.
   */
  @Override
  public void setYaw(double yaw) {
    imu.setYaw(yaw);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU, inverts them all if SwerveIMU is inverted.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray) {
    imu.getYawPitchRoll(yprArray);
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d() {
    double[] yprArray = new double[3];
    imu.getYawPitchRoll(yprArray);
    return new Rotation3d(yprArray[2], yprArray[1], yprArray[0]);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second quared. If acceleration
   * isn't supported returns empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel() {
    short[] initial = new short[3];
    imu.getBiasedAccelerometer(initial);
    return Optional.of(new Translation3d(initial[0], initial[1], initial[2]).times(9.81 / 16384.0));
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU() {
    return imu;
  }
}
