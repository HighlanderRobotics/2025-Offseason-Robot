package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setMotorVoltage(double voltage);

  public void setMotorPosition(Rotation2d targetPosition);
}
