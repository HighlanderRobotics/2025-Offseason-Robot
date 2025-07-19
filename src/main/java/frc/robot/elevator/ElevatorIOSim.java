package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          ElevatorSubsystem.GEAR_RATIO,
          Units.lbsToKilograms(7.0 + (3.25 / 2)),
          ElevatorSubsystem.DRUM_RADIUS_METERS,
          0.0,
          ElevatorSubsystem.MAX_EXTENSION_METERS,
          true,
          0.0);

  private double volts = 0.0;
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.0,
          0.06,
          (DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt * ElevatorSubsystem.DRUM_RADIUS_METERS)
              / ElevatorSubsystem.GEAR_RATIO);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(40.0, 0.0, 0.1, new Constraints(5.0, 10.0));

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    sim.update(0.02);
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = volts;
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    // inputs.tempCelsius = sim.getTemperature();
  }

  @Override
  public void setVoltage(final double volts) {
    this.volts = volts;
    sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setTarget(final double meters) {
    setVoltage(
        pid.calculate(sim.getPositionMeters(), meters) + ff.calculate(pid.getSetpoint().velocity));
  }

  @Override
  public void setCurrent(final double amps) {
    setVoltage(DCMotor.getKrakenX60Foc(2).getVoltage(amps, sim.getVelocityMetersPerSecond()));
  }

  @Override
  public void resetEncoder(final double position) {}
}
