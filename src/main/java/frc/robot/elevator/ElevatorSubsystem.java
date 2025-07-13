// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double GEAR_RATIO = 0; // TODO
  public static final double DRUM_RADIUS_METERS = 0; // TODO

  public enum ElevatorState {
    IDLE(0.0),
    PRE_INTAKE_CORAL_GROUND(0.0),
    INTAKE_CORAL_GROUND(0.0),
    L1(0.0),
    PRE_L2(0.0),
    L2(0.0),
    PRE_L3(0.0),
    L3(0.0),
    PRE_L4(0.0),
    L4(0.0),
    POST_L4(0.0),
    INTAKE_ALGAE_REEF_HIGH(0.0),
    INTAKE_ALGAE_REEF_LOW(0.0),
    INTAKE_ALGAE_GROUND(0.0),
    BARGE(0.0),
    PROCESSOR(0.0);

    private final double extensionMeters;

    private ElevatorState(double extensionMeters) {
      this.extensionMeters = extensionMeters;
    }

    public double getExtensionMeters() {
      return extensionMeters;
    }
  }

  private ElevatorState state = ElevatorState.IDLE;

  private double setpoint = 0.0;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Sets the elevator to the height corresponding to the current state of the robot
    // Arm, intake, and any future subsystems should more or less follow this same pattern
    // (additional logic might be needed though)
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    setExtension(() -> state.extensionMeters);
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public Command setExtension(DoubleSupplier meters) {
    return this.run(
        () -> {
          io.setPosition(meters.getAsDouble());
          setpoint = meters.getAsDouble();
        });
  }

  public boolean atExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }
}
