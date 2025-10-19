package frc.robot.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double leaderPositionMeters = 0.0;
    public double leaderVelocityMetersPerSec = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderVoltage = 0.0;
    public double leaderTempC = 0.0;

    public double followerPositionMeters = 0.0;
    public double followerVelocityMetersPerSec = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerVoltage = 0.0;
    public double followerTempC = 0.0;
  }

  void updateInputs(ElevatorIOInputs inputs);

  void setPositionSetpoint(double positionMeters, double acceleration);

  void setVoltage(double volts);

  void setCurrent(double amps);

  void resetEncoder(double position);

  default void stop() {
    setVoltage(0.0);
  }

  default void resetEncoder() {
    resetEncoder(0.0);
  }
}
