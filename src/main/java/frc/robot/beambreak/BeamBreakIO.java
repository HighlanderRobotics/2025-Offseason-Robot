package frc.robot.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public static class BeambreakIOInputs {
    public boolean get = false;
  }

  public void updateInputs(BeambreakIOInputsAutoLogged inputs);
}
