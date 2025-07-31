// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*** Works kinda the same as the arm */
public class IntakeSubsystem extends RollerSubsystem {
  public static final double GEAR_RATIO = (42.0 * 70.0) / (9.0 * 20.0);

  public enum IntakeState {
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    INTAKE(Rotation2d.fromDegrees(120), 10.0); // TODO ahhhh

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

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Setpoint")
  private Rotation2d setpoint = Rotation2d.kZero;

  // For sim
  private boolean bbSim = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io, RollerIO rollerIO) {
    super(rollerIO, "Intake");
    this.io = io;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    if (Robot.isSimulation()) Logger.recordOutput("Intake/Sim Canrange", bbSim);
  }

  public void setState(IntakeState state) {
    this.state = state;
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          io.setPivotAngle(target.get());
          setpoint = target.get();
          Logger.recordOutput("Intake/Setpoint", setpoint);
        });
  }

  public Command setStateAngleVoltage() { // i'll take awful method names for 500, alex
    return Commands.sequence(
        setPivotAngle(() -> state.getPivotAngle()),
        setRollerVoltage(() -> state.getRollerVoltage()));
  }

  public boolean atAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean hasCoral() {
    return Robot.isSimulation() ? bbSim : inputs.canrange;
  }

  public Rotation2d getAngle() {
    return Robot.isReal() ? inputs.cancoderPosition : inputs.motorPosition;
  }

  public void setSimBeambreak(boolean b) {
    if (Robot.isSimulation()) {
      bbSim = b;
    } else {
      return;
    }
  }
}
