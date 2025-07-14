// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.routing;

import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;

public class RoutingSubsystem extends RollerSubsystem {
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem(RollerIO rollerIO) {
    super(rollerIO, "Routing");
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
