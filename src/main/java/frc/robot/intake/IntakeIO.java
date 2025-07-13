// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public Rotation2d motorPosition = new Rotation2d();
    public Rotation2d cancoderPosition = new Rotation2d();
    public double voltage = 0.0;
    public double angularVelocityRPS = 0.0;
  }

  public void updateInputs(final IntakeIOInputs inputs);

  public void setVoltage(final double voltage);

  public void setMotorPosition(final Rotation2d position);

  public void setEncoderPosition(final Rotation2d position);
}
