package frc.robot.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public class RollerIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTemperatureCelsius = 0.0;
  }

  public void updateInputs(RollerIOInputs inputs);

  public void setRollerVoltage(double volts);

  public void setRollerVelocity(double velocityRPS);
}
