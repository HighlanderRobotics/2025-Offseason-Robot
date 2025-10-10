package frc.robot.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    public static final double GEAR_RATIO = 6.0 / 1.0;
    public static final double SPROCKET_DIAMETER_METERS = Units.inchesToMeters(1.257);
    public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(68.0);

    public static final double MAX_ACCELERATION = 10.0;

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private double currentFilterValue = 0.0;

    private boolean hasZeroed = false;

    public enum ElevatorState {
        IDLE(Units.inchesToMeters(6)),
        // coral
        READY_CORAL(Units.inchesToMeters(6)),
        PRE_INTAKE_CORAL_GROUND(Units.inchesToMeters(36)),
        INTAKE_CORAL_GROUND(Units.inchesToMeters(28)),
        L1(Units.inchesToMeters(25)),
        PRE_L2(Units.inchesToMeters(22)),
        L2(Units.inchesToMeters(22)),
        PRE_L3(Units.inchesToMeters(36)),
        L3(Units.inchesToMeters(36)),
        PRE_L4(Units.inchesToMeters(68.50)),
        L4(Units.inchesToMeters(61.5)),
        POST_L4(Units.inchesToMeters(61.5)),
        // algae
        INTAKE_ALGAE_REEF_HIGH(Units.inchesToMeters(53)),
        INTAKE_ALGAE_REEF_LOW(Units.inchesToMeters(36)),
        INTAKE_ALGAE_GROUND(Units.inchesToMeters(25)),
        BARGE(Units.inchesToMeters(61.5)),
        READY_ALGAE(Units.inchesToMeters(6)),
        PROCESSOR(Units.inchesToMeters(14)),
        // climbing
        PRE_CLIMB(Units.inchesToMeters(6)),
        CLIMB(Units.inchesToMeters(6));
    
        private final double extensionMeters;
    
        private ElevatorState(double extensionMeters) {
          this.extensionMeters = extensionMeters;
        }
    
        public double getExtensionMeters() {
          return extensionMeters;
        }
      }

    @AutoLogOutput(key = "Elevator/State")
    private ElevatorState state = ElevatorState.IDLE;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        currentFilterValue = currentFilter.calculate(inputs.leaderStatorCurrentAmps);
    }

    public void setState(ElevatorState newState) {
        this.state = newState;
    }

    public Command setExtensionMeters(DoubleSupplier meters) {
        return this.run(() -> {
            io.setPositionSetpoint(meters.getAsDouble());
            Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
        });
    }

    public Command setVoltage(DoubleSupplier volts) {
        return this.run(() -> {
            io.setVoltage(volts.getAsDouble());
        });
    }

    public Command runCurrentZeroing() {
        return this.run(() -> {
            io.setVoltage(-2.0);
        })
        .until(() -> Math.abs(currentFilterValue) > 50)
        .finallyDo((interrupted) -> {
            if (!interrupted) {
                io.resetEncoder();
                hasZeroed = true;
            }
        });
    }

    public boolean atExtension(double expected) {
        return MathUtil.isNear(expected, inputs.leaderPositionMeters, 0.05);
    }

    public double getExtensionMeters() {
        return inputs.leaderPositionMeters;
    }
}
