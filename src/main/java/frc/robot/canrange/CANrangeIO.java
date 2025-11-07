package frc.robot.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {

  @AutoLog
  public static class CANrangeIOInputs {
    public double distanceMeters = 0.0;
    public boolean isDetected = false;
  }

  public void updateInputs(CANrangeIOInputs inputs);
}
