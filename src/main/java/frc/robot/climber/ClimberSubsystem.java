// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {

  @AutoLogOutput(key = "Climber/State")
  private ClimberState state = ClimberState.IDLE;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  public enum ClimberState {
    // for position 0 is straight up and 90 is fully retracted (positive is CCW)
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    PRE_CLIMB(Rotation2d.fromDegrees(0), 1.0),
    CLIMB(Rotation2d.fromDegrees(90), 10.0),
    ;

    public final Rotation2d position;
    public final double volts;

    private ClimberState(Rotation2d position, double volts) {
      this.position = position;
      this.volts = volts;
    }
  }

  public void setState(ClimberState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
