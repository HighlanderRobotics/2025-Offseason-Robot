package frc.robot.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public Rotation2d yaw = new Rotation2d();
    // Could log pitch velocity and roll velocity as well but theres really no use
    public double yawVelocityRadPerSec = 0.0;
    public Rotation2d pitch = new Rotation2d();
    public Rotation2d roll = new Rotation2d();

    public boolean isConnected = false;
  }

  public void updateInputs(GyroIOInputs inputs);

  /**
   * Resets the gyro's position
   *
   * @param yaw the position to set the gyro to
   */
  public void setYaw(Rotation2d yaw);
}
