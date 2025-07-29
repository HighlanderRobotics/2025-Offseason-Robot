package frc.robot.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    public Rotation2d position = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  void updateInputs(ArmIOInputs inputs);

  void setMotorVoltage(double voltage);

  void setMotorPosition(Rotation2d targetPosition);

  void setRollerVoltage(double voltage);
}
