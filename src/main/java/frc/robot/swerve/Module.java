// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** "ModuleSubsystem" */
public class Module {

  public record ModuleConstants(
      int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  
  public Module(ModuleIO io) {
    this.io = io;
    
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(
        new StringBuilder("Swerve/").append(inputs.prefix).append(" Module").toString(), inputs);
  }

  /** Closed loop (for auto) - velocity based*/
  public SwerveModuleState runClosedLoop(SwerveModuleState state) {
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    //TODO afaict the force stuff is for auto only so i will revisit that
    io.setDriveVelocitySetpoint(state.speedMetersPerSecond);
    
    return state;
  }

  /** Open loop (for teleop) - voltage based*/
  public SwerveModuleState runOpenLoop(SwerveModuleState state) {
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    io.setDriveVoltage(state.speedMetersPerSecond * Math.cos(state.angle.minus(inputs.turnPosition).getRadians())); //TODO i don't really know how the units work out here: m/s -> volts??
    
    return state;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }
}
