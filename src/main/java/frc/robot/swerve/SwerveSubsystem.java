// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AutoAim;
import frc.robot.utils.FieldUtils.AlgaeIntakeTargets;
import frc.robot.utils.FieldUtils.L1Targets;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveSubsystem extends SubsystemBase {

  // we don't have multiple robots so i kinda don't care
  public static final SwerveConstants constants = new SwerveConstants();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    // return estimator.getEstimatedPosition();
    return Pose2d.kZero;
  }

  // public boolean isNearPose(Pose2d pose) {
  //   return
  // }

  public boolean isNearReef() {
    return getPose()
            .getTranslation()
            .minus(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? AutoAim.BLUE_REEF_CENTER
                    : AutoAim.RED_REEF_CENTER)
            .getNorm()
        < 3.25;
  }

  public boolean isNearL1Reef() { // TODO ??
    return L1Targets.getNearestLine(getPose()).getDistance(getPose().getTranslation()) > 0.3;
  }

  public boolean isNearProcessor() {
    return MathUtil.isNear(
            getPose().getX(),
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? AutoAim.BLUE_PROCESSOR_POS.getX()
                : AutoAim.RED_PROCESSOR_POS.getX(),
            2)
        || MathUtil.isNear(
            getPose().getY(),
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? AutoAim.BLUE_PROCESSOR_POS.getY()
                : AutoAim.RED_PROCESSOR_POS.getY(),
            2);
  }

  // TODO autoAimL23
  // my naming skills leave much to be desired
  public Command autoAimToL23() {
    // return AutoAim.autoAimWithIntermediatePose(
    //                 this,
    //                 () -> {
    //                   var twist = swerve.getVelocityFieldRelative().toTwist2d(0.3);
    //                   return CoralTargets.getHandedClosestTarget(
    //                       swerve
    //                           .getPose()
    //                           .plus(
    //                               new Transform2d(
    //                                   twist.dx, twist.dy, Rotation2d.fromRadians(twist.dtheta))),
    //                       driver.leftBumper().getAsBoolean());
    //                 },
    //                 // Keeps the robot off the reef wall until it's aligned side-side
    //                 new Transform2d(
    //                     AutoAim.INITIAL_REEF_KEEPOFF_DISTANCE_METERS, 0.0, Rotation2d.kZero));
    return Commands.none();
  }

  public boolean nearL23() {
    return AutoAim.isInToleranceCoral(getPose());
  }

  // TODO autoAimL1
  public Command autoAimToL1() {
    // return AutoAim.alignToLine(
    //                 this,
    //                 () ->
    //                     modifyJoystick(driver.getLeftY())
    //                         * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
    //                 () ->
    //                     modifyJoystick(driver.getLeftX())
    //                         * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
    //                 () -> L1Targets.getNearestLine(getPose()));
    return Commands.none();
  }

  public boolean nearL1() {
    return AutoAim.isInTolerance(
        getPose(),
        new Pose2d(
            L1Targets.getNearestLine(getPose()).nearest(getPose().getTranslation()),
            L1Targets.getNearestLine(getPose()).getRotation()));
  }

  // TODO autoAimL4
  public Command autoAimToL4() {
    return Commands.none();
  }

  // TODO isInToleranceL4
  public boolean nearL4() {
    return true;
  }

  public Command autoAimToOffsetAlgae() {
    // return AutoAim.translateToPose(
    //   this,
    //   () ->
    //       AlgaeIntakeTargets.getOffsetLocation(
    //           AlgaeIntakeTargets.getClosestTargetPose(getPose())));
    return Commands.none();
  }

  public boolean nearIntakeAlgaeOffsetPose() {
    // return AutoAim.isInTolerance(
    //   getPose(),
    //   AlgaeIntakeTargets.getOffsetLocation(
    //       AlgaeIntakeTargets.getClosestTargetPose(
    //           getPose())),
    //   getVelocityFieldRelative(),
    //   Units.inchesToMeters(1.0),
    //   Units.degreesToRadians(1.0));
    return true;
  }

  public Command approachAlgae() {
    // return AutoAim.approachAlgae(
    //   this,
    //   () -> AlgaeIntakeTargets.getClosestTargetPose(swerve.getPose()),
    //   1);
    return Commands.none();
  }

  public boolean nearAlgaeIntakePose() {
    return AutoAim.isInToleranceAlgaeIntake(getPose());
  }

  // TODO rename this lmao
  public boolean isNotMoving() {
    // return MathUtil.isNear(
    //   0,
    //   Math.hypot(
    //       getVelocityRobotRelative().vxMetersPerSecond,
    //       getVelocityRobotRelative()
    //           .vyMetersPerSecond),
    //   AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND)
    //   && MathUtil.isNear(
    //       0.0,
    //       getVelocityRobotRelative().omegaRadiansPerSecond,
    //       3.0);
    return true;
  }

  public Command autoAimToProcessor() {
    // return AutoAim.autoAimWithIntermediatePose(
    //                 this,
    //                 () -> getPose().nearest(AutoAim.PROCESSOR_POSES),
    //                 new Transform2d(
    //                     -(constants.getBumperLength() / 2) - 0.5,
    //                     0.0,
    //                     Rotation2d.kZero));
    return Commands.none();
  }

  public boolean nearProcessor() {
    return AutoAim.isInTolerance(
        getPose()
            .nearest(AutoAim.PROCESSOR_POSES)
            // Moves the target pose inside the field, with the bumpers
            // aligned with the wall
            .transformBy(
                new Transform2d(-(constants.getBumperLength() / 2), 0.0, Rotation2d.kZero)),
        getPose());
  }

  public Command autoAimToBarge() {
    // return AutoAim.translateToXCoord(
    //                 this,
    //                 () ->
    //                     Math.abs(getPose().getX() - AutoAim.BLUE_NET_X)
    //                             > Math.abs(getPose().getX() - AutoAim.RED_NET_X)
    //                         ? AutoAim.RED_NET_X
    //                         : AutoAim.BLUE_NET_X,
    //                 () ->
    //                     modifyJoystick(driver.getLeftX())
    //                         * ROBOT_HARDWARE.swerveConstants.getMaxLinearSpeed(),
    //                 () ->
    //                     (Math.abs(getPose().getX() - AutoAim.BLUE_NET_X)
    //                                 > Math.abs(getPose().getX() - AutoAim.RED_NET_X)
    //                             ? Rotation2d.kZero
    //                             : Rotation2d.k180deg)
    //                         .plus(Rotation2d.fromDegrees(20.0)));
    return Commands.none();
  }

  // TODO so this is not correct LMAO
  public boolean isNearBarge() {
    final var diff = getPose().minus(AlgaeIntakeTargets.getClosestTargetPose(getPose()));
    return MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(1.0))
        && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(1.0))
        && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 2.0);
  }
}
