// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {
    public Rotation2d positionRotations = Rotation2d.kZero;
    public double pivotVoltage = 0.0;
  }

  public void updateInputs(ClimberIOInputs inputs);

  public void setPosition(Rotation2d position);

  public void setPivotVoltage(double voltage);
}
