// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public enum IntakeState {
    IDLE(0.0), // this will not be the real number!! this is just a placeholder
    INTAKE_CORAL(10.0),
    OUTTAKE_CORAL(-10.0);

    public final double voltage;

    IntakeState(double voltage) {
      this.voltage = voltage;
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setState(IntakeState state) {
    this.state = state;
  }

  public void setMotorVoltage(double voltage) {
    io.setMotorVoltage(voltage);
  }

  public void setMotorPosition(Rotation2d targetPosition) {
    io.setMotorPosition(targetPosition);
  }
}
