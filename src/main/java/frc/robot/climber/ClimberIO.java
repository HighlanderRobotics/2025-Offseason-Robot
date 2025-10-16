package frc.robot.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        public Rotation2d pivotPosition = new Rotation2d();
        public double pivotVelocityRotPerSec = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotStatorCurrentAmps = 0.0;
        public double pivotSupplyCurrentAmps = 0.0;
        public double pivotTempC = 0.0;

        public double rollerVelocityRotPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerStatorCurrentAmps = 0.0;
        public double rollerSupplyCurrentAmps = 0.0;
        public double rollerTempC = 0.0;
    }

    void updateInputs(ClimberIOInputs inputs);

    void setPivotPosition(Rotation2d setpoint);

    void setPivotVoltage(double volts);

    void resetEncoder(Rotation2d position);

    void setRollerVelocity(double rotationsPerSecond);

    void setRollerVoltage(double volts);
}
