package frc.robot.swerve.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation simulation;

  public GyroIOSim(GyroSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    simulation.setRotation(yaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.yaw = simulation.getGyroReading();
    inputs.yawVelocityRadPerSec = simulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
  }
}
