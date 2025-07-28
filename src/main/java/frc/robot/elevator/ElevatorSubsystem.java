// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  // put constants here
  // TODO CHANGE THESE VALUES TO THE REAL ONES
  public static final double GEAR_RATIO = 12.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static final Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(90.0);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(68.50);

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO io;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;

  public boolean hasZeroed = false;

  private double setpoint = 0.0;

  private final SysIdRoutine voltageSysid;
  private final SysIdRoutine currentSysid;

  public enum ElevatorState {
    IDLE(Units.inchesToMeters(6)),
    //coral
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
    //algae
    INTAKE_ALGAE_REEF_HIGH(Units.inchesToMeters(53)),
    INTAKE_ALGAE_REEF_LOW(Units.inchesToMeters(36)),
    INTAKE_ALGAE_GROUND(Units.inchesToMeters(25)),
    BARGE(Units.inchesToMeters(61.5)),
    READY_ALGAE(Units.inchesToMeters(6)),
    PROCESSOR(Units.inchesToMeters(14)),
    //climbing
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

  /** Creates a new ElevatorSubsystem. */
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

  private ElevatorState state = ElevatorState.IDLE;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);

    // carriage.setLength(inputs.positionMeters);
    // if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Elevator/Mechanism2d", mech2d);

    // if (Robot.ROBOT_TYPE != RobotType.REAL)
    //   Logger.recordOutput("Elevator/Carriage Pose", getCarriagePose());

    // if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Elevator/Has Zeroed",
    // hasZeroed);
    // if (Robot.ROBOT_TYPE != RobotType.REAL)
    // Logger.recordOutput("Elevator/Filtered Current", currentFilterValue);
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setPositionTarget(meters.getAsDouble());
          setpoint = meters.getAsDouble();
          if (Robot.ROBOT_TYPE != RobotType.REAL)
            Logger.recordOutput("Elevator/Setpoint", setpoint);
        });
  }

  public Command setExtension(double meters) {
    return this.setExtension(() -> meters);
  }

  public Command setVoltage(double voltage) {
    return this.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return this.setVoltage(voltage.getAsDouble());
  }

  // public Command setCurrent(double amps) {
  //   return this.run(
  //       () -> {
  //         io.setCurrent(amps);
  //       });
  // }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              io.setVoltage(-2.0);
              setpoint = 0.0;
              if (Robot.ROBOT_TYPE != RobotType.REAL)
                Logger.recordOutput("Elevator/Setpoint", Double.NaN);
            })
        .until(() -> Math.abs(currentFilterValue) > 50.0)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                io.resetEncoder(0.0);
                hasZeroed = true;
              }
            });
  }

  public boolean isNearExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public boolean isNearExtension(double expected, double toleranceMeters) {
    return MathUtil.isNear(expected, inputs.positionMeters, toleranceMeters);
  }

  public Command runSysid() {
    final Function<SysIdRoutine, Command> runSysid =
        (routine) ->
            Commands.sequence(
                routine
                    .quasistatic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .quasistatic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)),
                Commands.waitUntil(() -> Math.abs(inputs.velocityMetersPerSec) < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kForward)
                    .until(() -> inputs.positionMeters > Units.inchesToMeters(50.0)),
                Commands.waitUntil(() -> inputs.velocityMetersPerSec < 0.1),
                routine
                    .dynamic(SysIdRoutine.Direction.kReverse)
                    .until(() -> inputs.positionMeters < Units.inchesToMeters(10.0)));
    return Commands.sequence(
        runCurrentZeroing(), runSysid.apply(voltageSysid), runSysid.apply(currentSysid));
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }
}
