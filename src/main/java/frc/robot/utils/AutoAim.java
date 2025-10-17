package frc.robot.utils;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class AutoAim {
  static final double MAX_ANGULAR_SPEED = 10.0;
  static final double MAX_ANGULAR_ACCELERATION = 10.0;
  static final double MAX_AUTOAIM_SPEED = 3.0;
  static final double MAX_AUTOAIM_ACCELERATION = 4.0;
  static final Constraints DEFAULT_TRANSLATIONAL_CONSTRAINTS =
      new Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION);
  static final Constraints DEFAULT_ANGULAR_CONSTRAINTS =
      new Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  public static final Translation2d RED_REEF_CENTER = ChoreoAllianceFlipUtil.flip(BLUE_REEF_CENTER);

  public static double BLUE_NET_X = 8.08 + Units.inchesToMeters(4);
  public static double RED_NET_X = ChoreoAllianceFlipUtil.flipX(BLUE_NET_X);

  public static Pose2d BLUE_PROCESSOR_POS = new Pose2d(5.973, 0, Rotation2d.fromDegrees(270));
  public static Pose2d RED_PROCESSOR_POS = ChoreoAllianceFlipUtil.flip(BLUE_PROCESSOR_POS);
  public static List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR_POS, RED_PROCESSOR_POS);

  public static final double L1_TROUGH_WIDTH_METERS = 0.935;

  public static final double TRANSLATION_TOLERANCE_METERS = Units.inchesToMeters(2.0);
  public static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);
  public static final double VELOCITY_TOLERANCE_METERSPERSECOND = 0.5;
  public static final double INITIAL_REEF_KEEPOFF_DISTANCE_METERS = -0.1;

  static final ProfiledPIDController VX_CONTROLLER =
      new ProfiledPIDController(
          10.0, 0.01, 0.02, new Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
  static final ProfiledPIDController VY_CONTROLLER =
      new ProfiledPIDController(
          10.0, 0.01, 0.02, new Constraints(MAX_AUTOAIM_SPEED, MAX_AUTOAIM_ACCELERATION));
  static final ProfiledPIDController HEADING_CONTROLLER =
      new ProfiledPIDController(
          6.0, 0.0, 0.0, new Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION));

  public static void resetPIDs(Pose2d robotPos, ChassisSpeeds robotVelocityFieldRelative) {
    VX_CONTROLLER.reset(robotPos.getX(), robotVelocityFieldRelative.vxMetersPerSecond);
    VY_CONTROLLER.reset(robotPos.getY(), robotVelocityFieldRelative.vyMetersPerSecond);
    HEADING_CONTROLLER.reset(
        robotPos.getRotation().getRadians(), robotVelocityFieldRelative.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds calculateSpeeds(Pose2d robotPose, Pose2d target) {
    VX_CONTROLLER.setConstraints(DEFAULT_TRANSLATIONAL_CONSTRAINTS);
    VY_CONTROLLER.setConstraints(DEFAULT_TRANSLATIONAL_CONSTRAINTS);
    HEADING_CONTROLLER.setConstraints(DEFAULT_ANGULAR_CONSTRAINTS);
    return new ChassisSpeeds(
        VX_CONTROLLER.calculate(robotPose.getX(), target.getX())
            + VX_CONTROLLER.getSetpoint().velocity,
        VY_CONTROLLER.calculate(robotPose.getY(), target.getY())
            + VY_CONTROLLER.getSetpoint().velocity,
        HEADING_CONTROLLER.calculate(
                robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians())
            + HEADING_CONTROLLER.getSetpoint().velocity);
  }

  public static ChassisSpeeds calculateSpeeds(
      Pose2d robotPose, Pose2d target, Constraints constraints) {
    VX_CONTROLLER.setConstraints(constraints);
    VY_CONTROLLER.setConstraints(constraints);
    // Should heading controller be set here as well? Or just translational?
    return new ChassisSpeeds(
        VX_CONTROLLER.calculate(robotPose.getX(), target.getX())
            + VX_CONTROLLER.getSetpoint().velocity,
        VY_CONTROLLER.calculate(robotPose.getX(), target.getY())
            + VY_CONTROLLER.getSetpoint().velocity,
        HEADING_CONTROLLER.calculate(
                robotPose.getRotation().getRadians(), target.getRotation().getRadians())
            + HEADING_CONTROLLER.getSetpoint().velocity);
  }

  public static double getClosestBargeXCoord(Pose2d pose) {
    return Math.abs(pose.getX() - AutoAim.BLUE_NET_X) < Math.abs(pose.getX() - AutoAim.RED_NET_X)
        ? AutoAim.BLUE_NET_X
        : AutoAim.RED_NET_X;
  }

  public static Rotation2d getClosestBargeRotation(Pose2d pose) {
    return (Math.abs(pose.getX() - AutoAim.BLUE_NET_X) > Math.abs(pose.getX() - AutoAim.RED_NET_X)
            ? Rotation2d.kZero
            : Rotation2d.k180deg)
        .plus(Rotation2d.fromDegrees(20.0));
  }
}
