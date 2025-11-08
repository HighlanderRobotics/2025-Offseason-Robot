package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class AutoAim {
  static final double MAX_ANGULAR_SPEED = 10.0;
  static final double MAX_ANGULAR_ACCELERATION = 10.0;
  static final double MAX_TRANSLATIONAL_SPEED = 3.0;
  static final double MAX_TRANSLATIONAL_ACCELERATION = 4.0;
  public static final Constraints DEFAULT_TRANSLATIONAL_CONSTRAINTS =
      new Constraints(MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_ACCELERATION);
  public static final Constraints DEFAULT_ANGULAR_CONSTRAINTS =
      new Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

  public static final double TRANSLATION_TOLERANCE_METERS = Units.inchesToMeters(1.0);
  public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);
  public static final double VELOCITY_TOLERANCE_METERSPERSECOND = 0.5;
  public static final double INITIAL_REEF_KEEPOFF_DISTANCE_METERS = -0.1;

  //   public static final double ALGAE_APPROACH_SPEED_METERS_PER_SECOND = 1.0;

  // Velocity controllers
  static final ProfiledPIDController VX_CONTROLLER =
      new ProfiledPIDController(
          10.0,
          0.01,
          0.02,
          new Constraints(MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_ACCELERATION));
  static final ProfiledPIDController VY_CONTROLLER =
      new ProfiledPIDController(
          10.0,
          0.01,
          0.02,
          new Constraints(MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_ACCELERATION));
  static final ProfiledPIDController HEADING_CONTROLLER =
      new ProfiledPIDController(
          6.0, 0.0, 0.0, new Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION));

  static {
    HEADING_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static void resetPIDControllers(
      Pose2d robotPose, ChassisSpeeds robotVelocityFieldRelative) {
    VX_CONTROLLER.reset(robotPose.getX(), robotVelocityFieldRelative.vxMetersPerSecond);
    VY_CONTROLLER.reset(robotPose.getY(), robotVelocityFieldRelative.vyMetersPerSecond);
    HEADING_CONTROLLER.reset(
        robotPose.getRotation().getRadians(), robotVelocityFieldRelative.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds calculateSpeeds(Pose2d robotPose, Pose2d target) {
    return calculateSpeeds(
        robotPose,
        target,
        DEFAULT_TRANSLATIONAL_CONSTRAINTS,
        DEFAULT_TRANSLATIONAL_CONSTRAINTS,
        DEFAULT_ANGULAR_CONSTRAINTS);
  }

  public static ChassisSpeeds calculateSpeeds(
      Pose2d robotPose,
      Pose2d target,
      Constraints xConstraints,
      Constraints yConstraints,
      Constraints headingConstraints) {
    VX_CONTROLLER.setConstraints(xConstraints);
    VY_CONTROLLER.setConstraints(yConstraints);
    HEADING_CONTROLLER.setConstraints(headingConstraints);

    ChassisSpeeds speeds;

    if (isInTolerance(
        robotPose, target, TRANSLATION_TOLERANCE_METERS, ROTATION_TOLERANCE_RADIANS)) {
      speeds = new ChassisSpeeds();
    } else {
      speeds =
          new ChassisSpeeds(
              VX_CONTROLLER.calculate(robotPose.getX(), target.getX())
                  + VX_CONTROLLER.getSetpoint().velocity,
              VY_CONTROLLER.calculate(robotPose.getY(), target.getY())
                  + VY_CONTROLLER.getSetpoint().velocity,
              HEADING_CONTROLLER.calculate(
                      robotPose.getRotation().getRadians(), target.getRotation().getRadians())
                  + HEADING_CONTROLLER.getSetpoint().velocity);
    }
    Logger.recordOutput(
        "AutoAim/Target Speeds Robot Relative",
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotPose.getRotation()));

    return speeds;
  }

  //   public static double getClosestBargeXCoord(Pose2d pose) {
  //     return Math.abs(pose.getX() - FieldUtils.BLUE_NET_X)
  //             < Math.abs(pose.getX() - FieldUtils.RED_NET_X)
  //         ? FieldUtils.BLUE_NET_X
  //         : FieldUtils.RED_NET_X;
  //   }

  //   public static Rotation2d getClosestBargeRotation(Pose2d pose) {
  //     return (Math.abs(pose.getX() - FieldUtils.BLUE_NET_X)
  //                 > Math.abs(pose.getX() - FieldUtils.RED_NET_X)
  //             ? Rotation2d.kCW_90deg
  //             : Rotation2d.kCCW_90deg)
  //         // TODO: TUNE
  //         .plus(Rotation2d.fromDegrees(20.0));
  //   }

  public static boolean isInTolerance(
      Pose2d current,
      Pose2d target,
      double translationalToleranceMeters,
      double angularToleranceRadians) {
    Transform2d diff = current.minus(target);
    return MathUtil.isNear(0.0, Math.hypot(diff.getX(), diff.getY()), translationalToleranceMeters)
        && MathUtil.isNear(
            target.getRotation().getRadians(),
            current.getRotation().getRadians(),
            angularToleranceRadians);
  }
}
