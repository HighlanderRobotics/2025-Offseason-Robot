package frc.robot.swerve.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.module.Module.ModuleConstants;

public abstract class SwerveConstants {

  public SwerveConstants() {}

  public abstract double getTrackWidthX();

  public abstract double getTrackWidthY();

  public abstract double getBumperWidth();

  public abstract double getBumperLength();

  public double getDriveBaseRadius() {
    return Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
  }

  public double getMaxAngularSpeed() {
    return getMaxLinearSpeed() / getDriveBaseRadius();
  }

  public abstract double getMaxLinearSpeed();

  public abstract double getMaxLinearAcceleration();

  public abstract double getDriveGearRatio();

  public abstract double getTurnGearRatio();

  /** Defaults to inverted ie Mk4i, Mk4n. */
  public boolean getTurnMotorInverted() {
    return true;
  }

  public double getDriveRotorToMeters() {
    return getDriveGearRatio() / (getWheelRadiusMeters() * 2 * Math.PI);
  }

  public double getWheelRadiusMeters() {
    return Units.inchesToMeters(2.0);
  }

  // Hardware constants
  public abstract ModuleConstants getFrontLeftModule();

  public abstract ModuleConstants getFrontRightModule();

  public abstract ModuleConstants getBackLeftModule();

  public abstract ModuleConstants getBackRightModule();

  public abstract int getGyroID();

  // Hardware configurations
  public abstract TalonFXConfiguration getDriveConfiguration();

  public abstract TalonFXConfiguration getTurnConfiguration(int cancoderID);

  public abstract CANcoderConfiguration getCancoderConfiguration(Rotation2d cancoderOffset);
}
