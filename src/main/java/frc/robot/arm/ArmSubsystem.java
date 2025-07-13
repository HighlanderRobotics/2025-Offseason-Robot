// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    IDLE(Rotation2d.fromDegrees(0)),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(180)),
    L1(new Rotation2d()),
    PRE_L2(new Rotation2d()),
    L2(new Rotation2d()),
    PRE_L3(new Rotation2d()),
    L3(new Rotation2d()),
    PRE_L4(new Rotation2d()),
    L4(new Rotation2d()),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(90)),
    INTAKE_ALGAE_GROUND(new Rotation2d()),
    BARGE(new Rotation2d()),
    PROCESSOR(Rotation2d.fromDegrees(90)),
    CLIMB(new Rotation2d());

    private final Rotation2d angle;

    private ArmState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  private Rotation2d setpoint = Rotation2d.kZero;

  private ArmState state = ArmState.IDLE;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    setAngle(() -> state.angle);
  }

  public void setState(ArmState state) {
    this.state = state;
  }

  public Command setAngle(Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          setpoint = target.get();
        });
  }

  public boolean atAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean hasCoral() {
    return true; // TODO
  }
}
