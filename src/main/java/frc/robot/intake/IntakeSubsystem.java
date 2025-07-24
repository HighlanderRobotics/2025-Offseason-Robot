// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/*** Works kinda the same as the arm */
public class IntakeSubsystem extends RollerSubsystem {

  public enum IntakeState {
    IDLE(new Rotation2d(), 0.0),
    INTAKE(new Rotation2d(), 0.0);

    private final Rotation2d pivotAngle;
    private final double rollerVoltage;

    private IntakeState(Rotation2d pivotAngle, double rollerVoltage) {
      this.pivotAngle = pivotAngle;
      this.rollerVoltage = rollerVoltage;
    }

    public Rotation2d getPivotAngle() {
      return pivotAngle;
    }

    public double getRollerVoltage() {
      return rollerVoltage;
    }
  }

  private IntakeState state = IntakeState.IDLE;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Rotation2d setpoint = Rotation2d.kZero;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io, RollerIO rollerIO) {
    super(rollerIO, "Intake");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setState(IntakeState state) {
    this.state = state;
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          io.setPivotAngle(target.get());
          setpoint = target.get();
        });
  }

  // TODO this is cooked apparently because they both require the arm but it's the default command
  // so i also can't just proxy it
  public Command setStateAngleVoltage() { // i'll take awful method names for 500, alex
    return Commands.sequence(
        setPivotAngle(() -> state.getPivotAngle()),
        setRollerVoltage(() -> state.getRollerVoltage()));
  }

  public boolean atAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean hasCoral() {
    return inputs.canrange;
  }
}
