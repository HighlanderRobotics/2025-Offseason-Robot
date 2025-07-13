// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double voltage = 0.0;
    public double velocityMetersPerSec = 0.0;
  }

  public void updateInputs(final ElevatorIOInputs inputs);

  public void setPosition(double positionMeters);
}
