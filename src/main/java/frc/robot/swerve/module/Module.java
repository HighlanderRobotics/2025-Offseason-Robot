package frc.robot.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

// A single module
public class Module {
  public record ModuleConstants(
      int id, String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io) {
    this.io = io;
  }

  // Updates and logs the IO layer inputs
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + inputs.prefix + " Module", inputs);
  }

  // Runs closed-loop
  public SwerveModuleState runVelocitySetpoint(SwerveModuleState state) {
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    io.setDriveSetpoint(state.speedMetersPerSecond);

    return state;
  }

  // Runs open-loop
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    // Optimize state based on current angle
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    io.setDriveVoltage(
        state.speedMetersPerSecond
            * Math.cos(state.angle.minus(inputs.turnPosition).getRadians()), // TODO: ask about this
        focEnabled);

    return state;
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }
}
