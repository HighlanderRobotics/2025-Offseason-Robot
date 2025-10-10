// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {

  @AutoLogOutput(key = "Climber/State")
  private ClimberState state = ClimberState.IDLE;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  public enum ClimberState {
    // for position 0 is straight up and 90 is fully retracted (positive is counterclockwise)
    IDLE(0, 0.0),
    PRE_CLIMB(0, 1.0),
    CLIMB(90, 10.0),
    ;

    public final Rotation2d position;
    public final double volts;

    private ClimberState(double positionDegrees, double volts) {
      this.position = Rotation2d.fromDegrees(positionDegrees);
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

  // TODO setStateAngleVoltage
  public Command setStateAngleVoltage() {
    return Commands.none();
  }

  // TODO atExtension
  public boolean atExtension() {
    return true;
  }
}
