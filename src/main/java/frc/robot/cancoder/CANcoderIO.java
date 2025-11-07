package frc.robot.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CANcoderIO {
  @AutoLog
  public static class CANcoderIOInputs {
    public Rotation2d cancoderPositionRotations = new Rotation2d();
  }

  public void updateInputs(CANcoderIOInputs inputs);
}
