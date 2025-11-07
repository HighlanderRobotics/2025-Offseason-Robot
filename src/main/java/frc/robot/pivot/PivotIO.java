package frc.robot.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  public void updateInputs(PivotIOInputs inputs);

  public void setMotorVoltage(double voltage);

  public default void setMotorPosition(Rotation2d targetPosition) {
    setMotorPosition(targetPosition, 0);
  }

  public void setMotorPosition(Rotation2d targetPosition, int slot);

  public void resetEncoder(Rotation2d targetPosition);
}
