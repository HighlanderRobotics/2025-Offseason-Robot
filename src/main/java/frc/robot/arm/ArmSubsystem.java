// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*** Works kinda the same as the intake */
public class ArmSubsystem extends RollerSubsystem {

  public enum ArmState {
    // i actually have no idea which direction is positive
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(180), 0.0),
    L1(Rotation2d.fromDegrees(202.171), 0.0),
    PRE_L2(Rotation2d.fromDegrees(360 - 128.445), 0.0),
    L2(Rotation2d.fromDegrees(330), 0.0),
    PRE_L3(Rotation2d.fromDegrees(360 - 128.445), 0.0),
    L3(Rotation2d.fromDegrees(330), 0.0),
    PRE_L4(Rotation2d.fromDegrees(360 - 36), 0.0),
    L4(Rotation2d.fromDegrees(360 - 90), 0.0),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(90), 0.0),
    INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(360 - 131), 0.0),
    BARGE(Rotation2d.fromDegrees(360 - 55), 0.0),
    PROCESSOR(Rotation2d.fromDegrees(90), 0.0),
    CLIMB(Rotation2d.fromDegrees(300), 0.0);

    private final Rotation2d pivotAngle;
    private final double rollerVoltage; // TODO big todo

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

  @AutoLogOutput(key = "Arm/Setpoint")
  private Rotation2d setpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Arm/State")
  private ArmState state = ArmState.IDLE;

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final BeambreakIO bbIO;
  private final BeambreakIOInputsAutoLogged bbInputs = new BeambreakIOInputsAutoLogged();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO armIO, RollerIO rollerIO, BeambreakIO bbIO) {
    super(rollerIO, "Arm");
    this.armIO = armIO;
    this.bbIO = bbIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    armIO.updateInputs(inputs);
    bbIO.updateInputs(bbInputs);
    Logger.processInputs("Arm", inputs);
    Logger.processInputs("Arm Beambreak", bbInputs);
  }

  public void setState(ArmState state) {
    this.state = state;
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          armIO.setPivotAngle(target.get());
          setpoint = target.get();
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
    return bbInputs.get;
  }
}
