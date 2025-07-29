// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {

  public enum ShoulderState {
    IDLE(new Rotation2d()); // this will not be the real number!! this is just a placeholder

    private final Rotation2d angle;

    private ShoulderState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  private ShoulderState state = ShoulderState.IDLE;

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setState(ShoulderState state) {
    this.state = state;
  }
}
