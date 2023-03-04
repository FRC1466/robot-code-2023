package swervelib.imu;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/** SwerveIMU interface for the Pigeon. */
public class PigeonSwerve extends SwerveIMU {

  /** Pigeon v1 IMU device. */
  WPI_PigeonIMU imu;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon, does not support CANBus.
   */
  public PigeonSwerve(int canid) {
    imu = new WPI_PigeonIMU(canid);
    SmartDashboard.putData(imu);
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
