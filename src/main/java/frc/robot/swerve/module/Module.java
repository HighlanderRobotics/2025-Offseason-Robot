package frc.robot.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

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

  // Use for automated driving (e.g. auto)
  public SwerveModuleState runClosedLoop(SwerveModuleState state) {
    state.optimize(getAngle());

    io.setTurnSetpoint(state.angle);
    io.setDriveSetpoint(state.speedMetersPerSecond);

    return state;
  }


  // Runs drive motor using feedforward control only
  // For teleop
  // Returns optimized state
  public SwerveModuleState runOpenLoop(SwerveModuleState state, boolean focEnabled) {
    state.optimize(getAngle());

    double volts = state.speedMetersPerSecond * 12 / Robot.ROBOT_HARDWARE.getSwerveConstants().getMaxLinearSpeed();

    runVoltageSetpoint(volts, state.angle, focEnabled);

    return state;
  }

  // Runs open-loop
  public SwerveModuleState runVoltageSetpoint(SwerveModuleState state, boolean focEnabled) {
    state.optimize(getAngle());

     // state.speedMetersPerSecond is NOT m/s, it's Volts the conversion happens in SwerveSubsystem
      // TODO: MAKE THE WEIRD UNIT STUFF LESS CONFUSING
    runVoltageSetpoint(state.speedMetersPerSecond, state.angle, focEnabled);

    return state;
  }

  private void runVoltageSetpoint(double volts, Rotation2d targetAngle, boolean focEnabled) {
    io.setTurnSetpoint(targetAngle);
    io.setDriveVoltage(
        volts
            * Math.cos(targetAngle.minus(inputs.turnPosition).getRadians()),
        focEnabled);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }
}
