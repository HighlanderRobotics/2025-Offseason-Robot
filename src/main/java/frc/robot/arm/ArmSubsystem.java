// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends RollerSubsystem {

  public enum ArmState {
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(180), 0.0),
    L1(new Rotation2d(), 0.0),
    PRE_L2(new Rotation2d(), 0.0),
    L2(new Rotation2d(), 0.0),
    PRE_L3(new Rotation2d(), 0.0),
    L3(new Rotation2d(), 0.0),
    PRE_L4(new Rotation2d(), 0.0),
    L4(new Rotation2d(), 0.0),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(90), 0.0),
    INTAKE_ALGAE_GROUND(new Rotation2d(), 0.0),
    BARGE(new Rotation2d(), 0.0),
    PROCESSOR(Rotation2d.fromDegrees(90), 0.0),
    CLIMB(new Rotation2d(), 0.0);

    private final Rotation2d pivotAngle;
    private final double rollerVoltage; //TODO big todo

    private ArmState(Rotation2d angle, double voltage) {
      this.pivotAngle = angle;
      this.rollerVoltage = voltage;
    }

    public Rotation2d getPivotAngle() {
      return pivotAngle;
    }

    public double getRollerVoltage() {
      return rollerVoltage;
    }
  }

  private Rotation2d setpoint = Rotation2d.kZero;

  private ArmState state = ArmState.IDLE;

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO armIO, RollerIO rollerIO) {
    super(rollerIO, "Arm");
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    setAngle(() -> state.pivotAngle);
  }

  public void setState(ArmState state) {
    this.state = state;
  }

  public Command setAngle(Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          armIO.setMotorPosition(target.get());
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
