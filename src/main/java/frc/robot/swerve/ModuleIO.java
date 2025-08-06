// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public String prefix = "";

    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;

    public Rotation2d turnPosition = new Rotation2d();
    public Rotation2d absoluteTurnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void setDriveVoltage(double voltage);

  public void setDriveVelocitySetpoint(double vel);

  public void setTurnVoltage(double voltage);

  public void setTurnSetpoint(Rotation2d rot);
}
