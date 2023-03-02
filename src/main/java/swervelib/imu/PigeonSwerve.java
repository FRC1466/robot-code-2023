package swervelib.imu;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SwerveIMU interface for the Pigeon.
 */
public class PigeonSwerve extends SwerveIMU
{

  /**
   * Pigeon v1 IMU device.
   */
  WPI_PigeonIMU imu;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon, does not support CANBus.
   */
  public PigeonSwerve(int canid)
  {
    imu = new WPI_PigeonIMU(canid);
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.configFactoryDefault();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    imu.setYaw(yaw);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU, inverts them all if SwerveIMU is inverted.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    imu.getYawPitchRoll(yprArray);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU.
   * 
   * @param accel Array which will be filled with {x, y, z} in m/s/s.
   */
  @Override
  public void getAccel(Double[] accel) {
      short[] initial = new short[3];
      imu.getBiasedAccelerometer(initial);
      accel[0] = initial[0] / 16384.0 * 9.81;
      accel[1] = initial[1] / 16384.0 * 9.81;
      accel[2] = initial[2] / 16384.0 * 9.81;
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
