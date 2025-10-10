// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  // TODO incorporate roller pivot stuff
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  /**
   * 0 for position is horizontal against bumper, positive is upwards.
    we're in real life!! use degrees.
    degrees -> Rotation2d gets handled in the constructor
    Positive voltage is intaking, negative is outtaking. (TODO)
   */
  public enum IntakeState {
    IDLE(130,0.0),
    INTAKE_CORAL(0, 10.0),
    READY_CORAL_INTAKE(130, 1.0),
    HANDOFF(130, -5.0),
    PRE_L1(90, 1.0),
    SCORE_L1(9, -5.0),
    CLIMB(0, 0.0);

    public final Rotation2d position;
    public final double volts;

    private IntakeState(double positionDegrees, double volts) {
      this.position = Rotation2d.fromDegrees(positionDegrees);
      this.volts = volts;
    }

    public Rotation2d getAngle() {
      return position;
    }

    public double getVolts() {
      return volts;
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

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.position.getDegrees(), 10.0);
  }

  // TODO setStateAngleVoltage
  public Command setStateAngleVoltage() {
    return Commands.none();
  }

  // TODO hasCoral
  public boolean hasCoral() {
    return true;
  }
}
