// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.routing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;

public class RoutingSubsystem extends RollerSubsystem {

  public enum RoutingState {
    IDLE(0.0),
    INTAKE(0.0),
    ANTIJAM(0.0);

    private final double voltage;

    private RoutingState(double voltage) {
      this.voltage = voltage;
    }

    private double getVoltage() {
      return voltage;
    }
  }

  private RoutingState state = RoutingState.IDLE;

  private final BeambreakIO bbIO;
  private final BeambreakIOInputsAutoLogged bbInputs = new BeambreakIOInputsAutoLogged();

  /**
   * Creates a new RoutingSubsystem. There is no RoutingIO/RoutingIOReal because this is basically
   * just rollers + a beambreak, so it doesn't need to inherit anything else
   */
  public RoutingSubsystem(RollerIO rollerIO, BeambreakIO bbIO) {
    super(rollerIO, "Routing");
    this.bbIO = bbIO;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public void setState(RoutingState state) {
    this.state = state;
  }

  public boolean hasCoral() {
    return bbInputs.get;
  }

  public Command setStateRollerVoltage() {
    return setRollerVoltage(() -> state.getVoltage());
  }
}
