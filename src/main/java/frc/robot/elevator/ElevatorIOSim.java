package frc.robot.subsystems.elevator;

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
            DCMotor.getKrakenX60Foc(2), 
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
    public void updateInputs(final ElevatorIOInputsAutologged inputs) {
        sim.update(0.02); 
        inputs.positionMeters = physicsSim.getPosition();
        inputs.velocityMetersPerSec = physicsSim.getVelocity();
        inputs.appliedVolts = volts;
        inputs.statorCurrentAmps = physicsSim.getStatorCurrent();
        inputs.tempCelsius = sim.getTemperature();
    }

    @Override
    public void setVoltage(final double volts) {
        volts = voltage;
        physicsSim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0))
    }

    @Override
    public void setTarget(final double meters) {
        setVoltage(
        pid.calculate(physicsSim.getPositionMeters(), meters)
            + ff.calculate(pid.getSetpoint().velocity));
    }

    @Override
    public void setCurrent(final double amps) {
        setVoltage(DCMotor.getKrakenX60Foc(2)
        .getVoltage(amps,  physicsSim.getVelocityMetersPerSecond()));
    }

    @Override
    public void resetEncoder(final double position) {
    }
}

