package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim physicsSim =
      new ElevatorSim(
          // DCMotor.getKrakenX60Foc(2),
          // for 2 kraken x44s
          new DCMotor(
              12.0, 4.05, 275, 1.4, 7530.0 / 60.0, 2), // not sure if this is supposed to be at 12v?
          ElevatorSubsystem.GEAR_RATIO,
          // Add half of first stage mass bc its on a 2:1 ratio compared to carriage
          // First stage weighs 3.345 lbs
          // Carriage weighs 8.863
          // Arm weighs 5.625
          Units.lbsToKilograms((3.345 / 2) + 8.863 + 5.625),
          ElevatorSubsystem.SPROCKET_DIAMETER_METERS / 2,
          0.0,
          ElevatorSubsystem.MAX_EXTENSION_METERS,
          true,
          0.0);
  private double volts = 0.0;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(110.0, 0.0, 0.0, new Constraints(5.0, 10.0));
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.24,
          0.56,
          (DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt
                  * (ElevatorSubsystem.SPROCKET_DIAMETER_METERS / 2))
              / ElevatorSubsystem.GEAR_RATIO);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    physicsSim.update(0.020);
    inputs.leaderPositionMeters = physicsSim.getPositionMeters();
    inputs.leaderVelocityMetersPerSec = physicsSim.getVelocityMetersPerSecond();
    inputs.leaderVoltage = volts;
    inputs.leaderStatorCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.leaderTempC = 20.0;

    // Assume leader and follower are in perfect sync
    inputs.followerPositionMeters = inputs.leaderPositionMeters;
    inputs.followerVelocityMetersPerSec = inputs.leaderVelocityMetersPerSec;
    inputs.followerVoltage = inputs.leaderVoltage;
    inputs.followerStatorCurrentAmps = inputs.leaderStatorCurrentAmps;
    inputs.followerTempC = inputs.leaderTempC;
  }

  @Override
  public void setPositionSetpoint(double meters, double acceleration) {
    setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
  }

  @Override
  public void setVoltage(final double voltage) {
    volts = voltage;
    physicsSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void setCurrent(double amps) {
    // We can't do current control in sim so this noops
  }

  @Override
  public void resetEncoder(final double position) {
    // sim always has a perfectly accurate encoder
  }
}
