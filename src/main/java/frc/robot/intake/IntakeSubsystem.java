// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  //TODO incorporate roller pivot stuff
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public enum IntakeState {
    // 0 for position is horizontal against bumper
    // positive voltage is intaking, negative is outtaking TBD?
    IDLE(Rotation2d.fromDegrees(110), 0.0), // this will not be the real number!! this is just a placeholder
    INTAKE_CORAL(Rotation2d.fromDegrees(0),10.0),
    PRE_HANDOFF(Rotation2d.fromDegrees(110), 1.0),
    HANDOFF(Rotation2d.fromDegrees(110), -5.0),
    PRE_L1(Rotation2d.fromDegrees(90), 1.0),
    SCORE_L1(Rotation2d.fromDegrees(90), -5.0),
    CLIMB(Rotation2d.fromDegrees(0), 0.0);


    public final Rotation2d position;
    public final double volts;

    private IntakeState(Rotation2d position, double volts) {
      this.position = position;
      this.volts = volts;
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
