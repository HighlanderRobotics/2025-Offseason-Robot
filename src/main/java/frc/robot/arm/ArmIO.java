package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public Rotation2d cancoderPosition = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(ArmIOInputs inputs);

  public void setMotorVoltage(double voltage);

  public void setMotorPosition(Rotation2d targetPosition);

  public void setRollerVoltage(double voltage);
}
