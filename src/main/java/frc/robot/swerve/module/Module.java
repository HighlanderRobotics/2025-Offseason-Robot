package frc.robot.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

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

  /** Returns the current turn angle of the module at normal sampling frequency. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters at normal sampling frequency. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns this modules prefix ie "Back Left" */
  public String getPrefix() {
    return inputs.prefix;
  }

  /**
   * Returns the current drive velocity of the module in meters per second withat normal sampling
   * frequency.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position) at normal sampling frequency. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity) at normal sampling frequency. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public void setTurnSetpoint(Rotation2d rotation) {
    io.setTurnSetpoint(rotation);
  }
  
}
