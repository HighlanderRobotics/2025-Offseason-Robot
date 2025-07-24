package frc.robot.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public void updateInputs(final ElevatorIOInputs inputs);

  public void setVoltage(final double volts);

  public void setPositionTarget(final double meters);

  public void setCurrent(final double amps);

  public default void stop() {
    setVoltage(0.0);
  }

  public void resetEncoder(final double position);

  public default void resetEncoder() {
    resetEncoder(0.0);
  }
}
