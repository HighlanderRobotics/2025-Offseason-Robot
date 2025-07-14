// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    public double voltage = 0.0;
    public double angularVelocityRPS = 0.0;
  }

  public void updateInputs(final RollerIOInputsAutoLogged inputs);

  public void setVoltage(final double voltage);
}
