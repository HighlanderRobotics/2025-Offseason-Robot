package frc.robot.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {

  @AutoLog
  public static class CANrangeIOInputs {
    public double distanceCm = 0.0;
  }

  public void updateInputs(CANrangeIOInputs inputs);
}
