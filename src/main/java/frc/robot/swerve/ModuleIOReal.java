// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.swerve.Module.ModuleConstants;

/** Add your docs here. */
public class ModuleIOReal implements ModuleIO {
  // not setting up odometry thread because that is Beyond the scope of this practice
  private final ModuleConstants constants;

  private final TalonFX turn;
  private final TalonFX drive;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  private final StatusSignal<Angle> absoluteTurnPosition;

  // why voltage and not torque current?
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVelocityTorqueCurrentFOC driveVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0.0).withEnableFOC(true);
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);

  public ModuleIOReal(ModuleConstants constants) {
    this.constants = constants;

    drive = new TalonFX(constants.driveID(), "*");
    turn = new TalonFX(constants.turnID(), "*");
    cancoder = new CANcoder(constants.cancoderID(), "*");

    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();

    turnPosition = turn.getPosition();
    turnVelocity = turn.getVelocity();

    absoluteTurnPosition = cancoder.getAbsolutePosition();

    // TODO config
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, turnPosition, turnVelocity, absoluteTurnPosition);
    
    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();

    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());

    inputs.absoluteTurnPosition = Rotation2d.fromRotations(absoluteTurnPosition.getValueAsDouble());
  }

  // TODO
  @Override
  public void setDriveVelocitySetpoint(double vel) {
    drive.setControl(driveVelocityControl.withVelocity(vel));
  }

  @Override
  public void setTurnSetpoint(Rotation2d rot) {
    turn.setControl(turnPID.withPosition(rot.getRotations()));
  }

  @Override
  public void setDriveVoltage(double voltage) {
    drive.setControl(driveVoltage.withOutput(voltage));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turn.setControl(turnVoltage.withOutput(voltage));
  }
}
