// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.routing;

import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;

public class RoutingSubsystem extends RollerSubsystem {

  private final BeambreakIO bbIO;
  private final BeambreakIOInputsAutoLogged bbInputs = new BeambreakIOInputsAutoLogged();

  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem(RollerIO rollerIO, BeambreakIO bbIO) {
    super(rollerIO, "Routing");
    this.bbIO = bbIO;
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
