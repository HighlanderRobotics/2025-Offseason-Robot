// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmIOInputs {
    /***Position as reported by the pivot motor, change this to change the position */
    public Rotation2d motorPosition = new Rotation2d();

    /***Position as reported by the cancoder on the carriage. More reliable for seeing where it is, can't change it here though*/
    public Rotation2d cancoderPosition = new Rotation2d();

    public double pivotVoltage = 0.0;
  }

  public void updateInputs(final ArmIOInputs inputs);

  /*** Sets the voltage for the pivot. Voltage for the rollers are handled in RollerIOReal. I think this should only be needed for zeroing? */
  public void setPivotVoltage(final double voltage);

  public void setPivotAngle(final Rotation2d position);

  public void setEncoderPosition(final Rotation2d position);
}
