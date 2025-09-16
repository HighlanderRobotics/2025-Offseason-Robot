package frc.robot.swerve.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.module.Module.ModuleConstants;

public abstract class SwerveConstants {

  public SwerveConstants() {}

  /** The width from wheel to wheel in the x direction */
  public abstract double getTrackWidthX();

  /** The width from wheel to whel in the y direction */
  public abstract double getTrackWidthY();

  /** The side-to-side width of the robot from the edges of the bumpers */
  public abstract double getBumperWidth();

  /** The front-to-back width of the robot from the edges of the bumpers */
  public abstract double getBumperLength();

  /** The radius of the circle which fully encloses the drive base */
  public double getDriveBaseRadius() {
    return Math.hypot(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0);
  }

  /** The max speed the robot can spin at */
  public double getMaxAngularSpeed() {
    return getMaxLinearSpeed() / getDriveBaseRadius();
  }

  /** The max speed the robot can move laterally */
  public abstract double getMaxLinearSpeed();

  /** The max acceleration the drivetrain can create laterally */
  public abstract double getMaxLinearAcceleration();

  /** The gear ratio between the wheel and the drive motor */
  public abstract double getDriveGearRatio();

  /** The gear ratio between the wheel and the turn motor */
  public abstract double getTurnGearRatio();

  /**
   * Returns an array of module translations. The translations are relative to the robot's center.
   */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, getTrackWidthY() / 2.0),
      new Translation2d(-getTrackWidthX() / 2.0, -getTrackWidthY() / 2.0)
    };
  }

  /** Defaults to inverted ie Mk4i, Mk4n. */
  public boolean getTurnMotorInverted() {
    return true;
  }

  /** Converts drive motor rotations to meters of ground movement */
  public double getDriveRotorToMeters() {
    return getDriveGearRatio() / (getWheelRadiusMeters() * 2 * Math.PI);
  }

  /** Radius of the wheels used in the swerve modules */
  public double getWheelRadiusMeters() {
    return Units.inchesToMeters(2.0);
  }

  // Hardware constants
  public abstract ModuleConstants getFrontLeftModule();

  public abstract ModuleConstants getFrontRightModule();

  public abstract ModuleConstants getBackLeftModule();

  public abstract ModuleConstants getBackRightModule();

  /** The CAN id of the Pigeon2 */
  public abstract int getGyroID();

  // Hardware configurations
  /** The motor configuration for all of the drive motors */
  public abstract TalonFXConfiguration getDriveConfiguration();

  /**
   * The configuration for the turn motors, using the passed-in cancoder
   *
   * @param cancoderID the cancoder attached to the motor's swerve module
   * @return the configuration
   */
  public abstract TalonFXConfiguration getTurnConfiguration(int cancoderID);

  /**
   * The configuration for a swerve-module cancoder
   *
   * @param cancoderOffset the offset for the specific swerve module
   * @return the configuration
   */
  public abstract CANcoderConfiguration getCancoderConfiguration(Rotation2d cancoderOffset);
}
