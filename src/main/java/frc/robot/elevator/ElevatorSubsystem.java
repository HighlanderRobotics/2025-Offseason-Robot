package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
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
    // Although the motor takes it in terms of meters, we usually measure extension in terms of
    // inches
    // So the constructor handles the conversion
    IDLE(0),
    HANDOFF(37.841),
    INTAKE_CORAL_STACK(0),
    // coral
    PRE_L2(0),
    L2(15),
    PRE_L3(25),
    L3(29),
    PRE_L4(58.75),
    L4(52),
    // algae
    INTAKE_ALGAE_REEF_HIGH(43),
    INTAKE_ALGAE_REEF_LOW(26),
    INTAKE_ALGAE_STACK(10),
    INTAKE_ALGAE_GROUND(25),
    READY_ALGAE(0),

    BARGE(58.75),
    PROCESSOR(4),
    // climbing
    PRE_CLIMB(0),
    CLIMB(0);

    private final double extensionMeters;

    private ElevatorState(double extensionInches) {
      this.extensionMeters = Units.inchesToMeters(extensionInches);
    }

    public double getExtensionMeters() {
      return extensionMeters;
    }
  }

  @AutoLogOutput(key = "Elevator/State")
  private ElevatorState state = ElevatorState.IDLE;

  private double setpoint = 0.0;

  private final SysIdRoutine voltageSysid;
  private final SysIdRoutine currentSysid;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    voltageSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdTestStateVolts", state.toString())),
            new Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    currentSysid =
        new SysIdRoutine(
            new Config(
                Volts.of(30.0).per(Second),
                Volts.of(120.0),
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdTestStateCurrent", state.toString())),
            new Mechanism((volts) -> io.setCurrent(volts.in(Volts)), null, this));
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
    return this.run(
        () -> {
          io.setPositionSetpoint(meters.getAsDouble());
          setpoint = meters.getAsDouble();
          Logger.recordOutput("Elevator/Setpoint", meters.getAsDouble());
        });
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-2.0);
            })
        .until(() -> Math.abs(currentFilterValue) > 50)
        .finallyDo(
            (interrupted) -> {
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

  public Command runSysid() {
    final Function<SysIdRoutine, Command> runSysid =
        (routine) ->
            Commands.sequence(
                routine
                    .quasistatic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.leaderPositionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.leaderVelocityMetersPerSec < 0.1),
                routine
                    .quasistatic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.leaderPositionMeters < Units.inchesToMeters(10.0)),
                Commands.waitUntil(() -> Math.abs(inputs.leaderVelocityMetersPerSec) < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.leaderPositionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.leaderVelocityMetersPerSec < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.leaderPositionMeters < Units.inchesToMeters(10.0)));
    return Commands.sequence(
        runCurrentZeroing(), runSysid.apply(voltageSysid), runSysid.apply(currentSysid));
  }


  public boolean atExtension() {
    return atExtension(setpoint);
  }

  public Command setStateExtension() {
    return setExtensionMeters(() -> state.getExtensionMeters());
  }
}
