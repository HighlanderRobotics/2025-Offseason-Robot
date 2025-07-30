// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*** Works kinda the same as the intake */
public class ArmSubsystem extends RollerSubsystem {
  public static final double GEAR_RATIO = (44.0 / 16.0) * 23.0;

  public enum ArmState {
    // i actually have no idea which direction is positive
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(-180), -1.0),
    PRE_L1(Rotation2d.fromDegrees(-157), 0.0),
    L1(Rotation2d.fromDegrees(-157), 1.0),
    PRE_L2(Rotation2d.fromDegrees(-30), 0.0),
    L2(Rotation2d.fromDegrees(-52), 1.0),
    PRE_L3(Rotation2d.fromDegrees(-30), 0.0),
    L3(Rotation2d.fromDegrees(-52), 1.0),
    PRE_L4(Rotation2d.fromDegrees(-36), 0.0),
    L4(Rotation2d.fromDegrees(-90), 1.0),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(-90), -1.0),
    INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(-131), -1.0),
    PRE_BARGE(Rotation2d.fromDegrees(-55), 0.0),
    BARGE(Rotation2d.fromDegrees(-55), 1.0),
    PRE_PROCESSOR(Rotation2d.fromDegrees(-90), 0.0),
    PROCESSOR(Rotation2d.fromDegrees(-90), 1.0),
    CLIMB(Rotation2d.fromDegrees(-60), 0.0);

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

  // For sim
  private boolean bbSim = false;

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
    Logger.processInputs("Arm/Beambreak", bbInputs);
    if (Robot.isSimulation()) Logger.recordOutput("Arm/Sim Beambreak", bbSim);
  }

  public void setState(ArmState state) {
    this.state = state;
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          armIO.setPivotAngle(target.get());
          setpoint = target.get();
          Logger.recordOutput("Arm/Setpoint", setpoint);
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

  public boolean hasPiece() {
    return Robot.isSimulation() ? bbSim : bbInputs.get;
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
