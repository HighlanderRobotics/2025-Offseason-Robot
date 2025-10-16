// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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
    public final double rollerVolts;

    private ClimberState(double positionDegrees, double rollerVolts) {
      this.position = Rotation2d.fromDegrees(positionDegrees);
      this.rollerVolts = rollerVolts;
    }
  }

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  public void setState(ClimberState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setStateAngleVoltage() {
    return this.run(() -> {
      io.setPivotPosition(state.position);
      io.setRollerVoltage(state.rollerVolts);
    });
  }

  public boolean atExtension() {
    return MathUtil.isNear(state.position.getDegrees(), inputs.pivotPosition.getDegrees(), 5);
  }
}
