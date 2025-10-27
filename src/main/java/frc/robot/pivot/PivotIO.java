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

  public void setMotorPosition(Rotation2d targetPosition);

  public void resetEncoder(Rotation2d targetPosition);
}
