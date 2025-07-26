// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends RollerSubsystem {

  public enum ClimberState {
    IDLE(new Rotation2d(), 0.0),
    PRE_CLIMB(new Rotation2d(), 0.0),
    CLIMB(new Rotation2d(), 0.0);

    private Rotation2d pivotAngle;
    private double rollerVoltage;

    private ClimberState(Rotation2d pivotAngle, double rollerVoltage) {
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

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.IDLE;

  private Rotation2d setpoint = Rotation2d.kZero;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(RollerIO rollerIO, ClimberIO io) {
    super(rollerIO, "Climber");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setPivotAngle(Rotation2d position) {
    return this.run(() -> io.setPosition(position));
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          io.setPosition(target.get());
          setpoint = target.get();
        });
  }

  // this is literally the same mechanism three times but atp it's going to take me more time to
  // abstract it lmaooo
  // don't be like me, kids. plan out your code in advance 💀
  public Command setStateAngleVoltage() { // i'll take awful method names for 500, alex
    return Commands.sequence(
        setPivotAngle(() -> state.getPivotAngle()),
        setRollerVoltage(() -> state.getRollerVoltage()));
  }

  public void setState(ClimberState state) {
    this.state = state;
  }
}
