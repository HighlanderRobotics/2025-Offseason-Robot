// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;

public class IntakeSubsystem extends RollerSubsystem {

  public enum IntakeState {
    IDLE(new Rotation2d()),
    INTAKE(new Rotation2d());

    private final Rotation2d angle;

    private IntakeState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  private IntakeState state = IntakeState.IDLE;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io, RollerIO rollerIO) {
    super(rollerIO, "Intake");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.processInputs("Intake Beambreak", inputs);
  }

  public void setState(IntakeState state) {
    this.state = state;
  }

  public boolean atAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean hasCoral() {
    return io.hasCoral();
  }
}
