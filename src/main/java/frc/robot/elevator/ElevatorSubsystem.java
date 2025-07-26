// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double GEAR_RATIO = 2.5 / 1.0; // TODO
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0); // TODO
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(68.50);

  public enum ElevatorState {
    // i don't trust l4 or barge one bit
    IDLE(0.0),
    PRE_INTAKE_CORAL_GROUND(Units.inchesToMeters(29.941)),
    INTAKE_CORAL_GROUND(Units.inchesToMeters(18.103)),
    L1(Units.inchesToMeters(21.523)),
    PRE_L2(Units.inchesToMeters(11.984)),
    L2(Units.inchesToMeters(11.5)),
    PRE_L3(Units.inchesToMeters(26.852)),
    L3(Units.inchesToMeters(25.8)),
    PRE_L4(Units.inchesToMeters(61.5)),
    L4(Units.inchesToMeters(55)),
    POST_L4(Units.inchesToMeters(43.405)),
    INTAKE_ALGAE_REEF_HIGH(Units.inchesToMeters(45)),
    INTAKE_ALGAE_REEF_LOW(Units.inchesToMeters(28)),
    INTAKE_ALGAE_GROUND(Units.inchesToMeters(18.8)),
    BARGE(Units.inchesToMeters(61.5)),
    PROCESSOR(Units.inchesToMeters(7));

    private final double extensionMeters;

    private ElevatorState(double extensionMeters) {
      this.extensionMeters = extensionMeters;
    }

    public double getExtensionMeters() {
      return extensionMeters;
    }
  }

  @AutoLogOutput(key = "Elevator/State")
  public ElevatorState state = ElevatorState.IDLE;

  private ElevatorState prevState = ElevatorState.IDLE;

  @AutoLogOutput(key = "Elevator/Setpoint")
  private double setpoint = 0.0;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setPosition(meters.getAsDouble());
          setpoint = meters.getAsDouble();
          Logger.recordOutput("Elevator/Setpoint", setpoint);
        });
  }

  public Command setStateExtension() {
    return setExtension(() -> state.getExtensionMeters());
  }

  public boolean atExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }
}
