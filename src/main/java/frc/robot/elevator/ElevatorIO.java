import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    pulic static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    public void updateInputs(final ElevatorIOInputsAutologged inputs);

    public void setVoltage(final double volts);

    public void setTarget(final double meters);

    public void setCurrent(final double amps);

    public default void stop() {
        setVoltage(0.0);
    }

    public void resetEncoder(final double position);

    public default void resetEncoder() {
        resetEncoder(0.0);
    }
}
