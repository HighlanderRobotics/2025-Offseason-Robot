package frc.robot.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.camera.Camera;
import frc.robot.camera.CameraIOReal;
import frc.robot.camera.CameraIOSim;
import frc.robot.swerve.constants.OffseasonBotSwerveConstants;
import frc.robot.swerve.constants.SwerveConstants;
import frc.robot.swerve.gyro.GyroIO;
import frc.robot.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.swerve.gyro.GyroIOReal;
import frc.robot.swerve.gyro.GyroIOSim;
import frc.robot.swerve.module.Module;
import frc.robot.swerve.module.ModuleIOReal;
import frc.robot.swerve.module.ModuleIOSim;
import frc.robot.swerve.odometry.OdometryThreadIO;
import frc.robot.swerve.odometry.OdometryThreadIO.OdometryThreadIOInputs;
import frc.robot.swerve.odometry.PhoenixOdometryThread;
import frc.robot.swerve.odometry.PhoenixOdometryThread.Samples;
import frc.robot.swerve.odometry.PhoenixOdometryThread.SignalID;
import frc.robot.swerve.odometry.PhoenixOdometryThread.SignalType;
import frc.robot.utils.AutoAim;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.L1Targets;
import frc.robot.utils.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  public static final SwerveConstants SWERVE_CONSTANTS = new OffseasonBotSwerveConstants();

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Camera[] cameras;
  private final Pose3d[] cameraPoses;
  private final OdometryThreadIO odometryThread;
  private final OdometryThreadIOInputs odometryThreadInputs = new OdometryThreadIOInputs();
  private double lastOdometryUpdateTimestamp = 0.0;

  private SwerveDriveKinematics kinematics;

  private SwerveDrivePoseEstimator estimator;

  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d rawGyroRotation = new Rotation2d();

  private static final SignalID GYRO_SIGNAL_ID =
      new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID);
  private static final SignalID[] DRIVE_SIGNAL_IDS = {
    new SignalID(SignalType.DRIVE, 0),
    new SignalID(SignalType.DRIVE, 1),
    new SignalID(SignalType.DRIVE, 2),
    new SignalID(SignalType.DRIVE, 3)
  };
  private static final SignalID[] TURN_SIGNAL_IDS = {
    new SignalID(SignalType.TURN, 0),
    new SignalID(SignalType.TURN, 1),
    new SignalID(SignalType.TURN, 2),
    new SignalID(SignalType.TURN, 3)
  };

  private final SwerveDriveSimulation swerveSimulation;

  private Alert usingSyncOdoAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);

  boolean hasFrontTags = false;

  public SwerveSubsystem(SwerveDriveSimulation swerveSimulation) {
    if (Robot.ROBOT_TYPE == RobotType.SIM) {
      // Add simulated modules
      modules =
          new Module[] {
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontLeftModuleConstants(),
                    swerveSimulation.getModules()[0])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontRightModuleConstants(),
                    swerveSimulation.getModules()[1])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackLeftModuleConstants(),
                    swerveSimulation.getModules()[2])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackRightModuleConstants(),
                    swerveSimulation.getModules()[3]))
          };
      // cameras =
      //     Arrays.stream(SWERVE_CONSTANTS.getCameraConstants())
      //         .map((constants) -> new Camera(new CameraIOSim(constants)))
      //         .toArray(Camera[]::new);
      // TODO something's cooked here
      cameras =
          new Camera[] {
            new Camera(new CameraIOSim(SWERVE_CONSTANTS.getCameraConstants()[0])),
            new Camera(new CameraIOSim(SWERVE_CONSTANTS.getCameraConstants()[1])),
            new Camera(new CameraIOSim(SWERVE_CONSTANTS.getCameraConstants()[2])),
            new Camera(new CameraIOSim(SWERVE_CONSTANTS.getCameraConstants()[3]))
          };
    } else {
      // Add real modules
      modules =
          new Module[] {
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontLeftModuleConstants())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontRightModuleConstants())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackLeftModuleConstants())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackRightModuleConstants()))
          };
      // cameras =
      //     Arrays.stream(SWERVE_CONSTANTS.getCameraConstants())
      //         .map((constants) -> new Camera(new CameraIOReal(constants)))
      //         .toArray(Camera[]::new);

      cameras =
          new Camera[] {
            new Camera(new CameraIOReal(SWERVE_CONSTANTS.getCameraConstants()[0])),
            new Camera(new CameraIOReal(SWERVE_CONSTANTS.getCameraConstants()[1])),
            new Camera(new CameraIOReal(SWERVE_CONSTANTS.getCameraConstants()[2])),
            new Camera(new CameraIOReal(SWERVE_CONSTANTS.getCameraConstants()[3]))
          };
    }

    this.cameraPoses = new Pose3d[cameras.length];
    // needed to prevent NPE
    for (int i = 0; i < cameras.length; i++) {
      cameraPoses[i] = Pose3d.kZero;
    }

    this.gyroIO =
        Robot.ROBOT_TYPE != RobotType.SIM
            ? new GyroIOReal(SWERVE_CONSTANTS.getGyroID(), SWERVE_CONSTANTS.getGyroConfig())
            : new GyroIOSim(swerveSimulation.getGyroSimulation());

    this.swerveSimulation = swerveSimulation;

    this.kinematics = new SwerveDriveKinematics(SWERVE_CONSTANTS.getModuleTranslations());
    // Std devs copied from reefscape
    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.6, 0.6, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.4));

    this.odometryThread = PhoenixOdometryThread.getInstance();
  }

  @Override
  public void periodic() {
    Tracer.trace(
        "Swerve Periodic",
        () -> {
          Tracer.trace(
              "Update odo thread inputs",
              () -> odometryThread.updateInputs(odometryThreadInputs, lastOdometryUpdateTimestamp));
          Logger.processInputs("AsyncOdo", odometryThreadInputs);
          if (!odometryThreadInputs.sampledStates.isEmpty()) {
            lastOdometryUpdateTimestamp =
                odometryThreadInputs
                    .sampledStates
                    .get(odometryThreadInputs.sampledStates.size() - 1)
                    .timestamp();
          }

          Tracer.trace("Update gyro inputs", () -> gyroIO.updateInputs(gyroInputs));
          Logger.processInputs("Swerve/Gyro", gyroInputs);

          for (Module module : modules) {
            Tracer.trace("Update module inputs for " + module.getPrefix(), module::periodic);
          }

          for (Camera camera : cameras) {
            Tracer.trace("Camera " + camera.getName() + " Periodic", camera::periodic);
          }

          Tracer.trace("Update odometry", this::updateOdometry);
          Tracer.trace("Update vision", this::updateVision);
        });
  }

  private void updateOdometry() {

    List<Samples> sampledStates = odometryThreadInputs.sampledStates;
    // Use sync samples if there aren't any async ones
    if (sampledStates.size() == 0
        || Robot.isSimulation()
        || sampledStates.get(0).values().isEmpty()) {
      usingSyncOdoAlert.set(true);
      sampledStates = getSyncSamples();
    } else {
      usingSyncOdoAlert.set(false);
    }
    // Update for each set of samples
    for (Samples sample : sampledStates) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      boolean hasNullModulePosition = false;
      boolean hasNullGyroRotation = false;
      // Get the positions and deltas for each module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        Double dist = sample.values().get(DRIVE_SIGNAL_IDS[moduleIndex]);
        if (dist == null) {
          // No value at timestamp
          hasNullModulePosition = true;
          break;
        }

        Double rot = sample.values().get(TURN_SIGNAL_IDS[moduleIndex]);
        if (rot == null) {
          hasNullModulePosition = true;
          break;
        }

        // All data exists at this timestamp
        modulePositions[moduleIndex] =
            new SwerveModulePosition(dist, Rotation2d.fromRotations(rot)); // Values from thread
        // Change since last sample
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle.minus(lastModulePositions[moduleIndex].angle));

        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      hasNullGyroRotation =
          !gyroInputs.isConnected || (sample.values().get(GYRO_SIGNAL_ID) == null);

      // Set DS alerts
      missingModuleData.set(hasNullModulePosition);
      missingGyroData.set(hasNullGyroRotation);

      if (hasNullModulePosition && hasNullGyroRotation) {
        // Can't really do anything else rn bc theres no data
        continue;
      } else if (hasNullModulePosition && !hasNullGyroRotation) {
        rawGyroRotation = Rotation2d.fromDegrees(sample.values().get(GYRO_SIGNAL_ID));

        // If we're missing data, just update with the gyro and the previous module positions
        estimator.updateWithTime(sample.timestamp(), rawGyroRotation, lastModulePositions);
        continue;
      } else if (!hasNullModulePosition && hasNullGyroRotation) {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        // If theres no gyro data, update the rotation with the change in position
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      } else if (!hasNullModulePosition && !hasNullGyroRotation) {
        // We have all of our data
        rawGyroRotation = Rotation2d.fromDegrees(sample.values().get(GYRO_SIGNAL_ID));
      }

      // Apply update
      estimator.updateWithTime(sample.timestamp(), rawGyroRotation, modulePositions);
    }
  }

  private void updateVision() {
    hasFrontTags = false;
    for (int i = 0; i < cameras.length; i++) {
      if (cameras[i].hasFrontTags()) hasFrontTags = true;
      cameras[i].updateCamera(estimator);
      cameraPoses[i] = cameras[i].getPose();
    }
    // Logger.recordOutput("Vision/Front Cameras Have Tags", hasFrontTags);
    // TODO
    // if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Vision/Camera Poses",
    // cameraPoses);
    Pose3d[] arr = new Pose3d[cameras.length];
    for (int k = 0; k < cameras.length; k++) {
      arr[k] = getPose3d().transformBy(cameras[k].getCameraConstants().robotToCamera());
    }
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Vision/Camera Poses on Robot", arr);
  }

  /**
   * Runs the modules to the specified ChassisSpeeds (robot velocity)
   *
   * @param speeds the ChassisSpeeds to run the drivetrain at
   * @param openLoop boolean for if the drivetrain should run with feedforward control (open loop)
   *     or with feedback control (closed loop)
   */
  private void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Converts time continuous chassis speeds to setpoints after the specified time (dtSeconds)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert drivetrain setpoint into individual module setpoints
    final SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    // Makes sure each wheel isn't asked to go above its max. Recalcs the states if needed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVE_CONSTANTS.getMaxLinearSpeed());
    Logger.recordOutput("SwerveStates/Setpoints", states);

    Logger.recordOutput("Swerve/Target Speeds", speeds);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];

    for (int i = 0; i < optimizedStates.length; i++) {
      if (openLoop) {
        // Heuristic to enable/disable FOC
        // enables FOC if the robot is moving at 90% of drivetrain max speed
        final boolean focEnable =
            Math.sqrt(
                    Math.pow(this.getVelocityRobotRelative().vxMetersPerSecond, 2)
                        + Math.pow(this.getVelocityRobotRelative().vyMetersPerSecond, 2))
                < SWERVE_CONSTANTS.getMaxLinearSpeed() * 0.9; // 0.9 is 90% of drivetrain max speed
        optimizedStates[i] = modules[i].runOpenLoop(states[i], focEnable);
      } else {
        optimizedStates[i] = modules[i].runClosedLoop(states[i]);
      }
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
  }

  /**
   * Drive closed-loop at robot relative speeds
   *
   * @param speeds robot relative speed setpoint
   * @return a command driving to target speeds
   */
  public Command driveClosedLoopRobotRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), false));
  }

  /**
   * Drive closed-loop at field relative speeds (i.e. for autoaim)
   *
   * @param speeds
   * @return a Command driving to the target speeds
   */
  public Command driveClosedLoopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()), false));
  }

  /**
   * Drive closed-loop at field relative speeds (i.e. for autoaim)
   *
   * @param speeds
   * @return a Command driving to the target speeds
   */
  public Command driveOpenLoopRobotRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), true));
  }

  /**
   * Drives open-loop. Speeds field relative to driver. Used for teleop
   *
   * @param speeds the field-relative speeds to drive at
   * @return a Command driving at those speeds
   */
  public Command driveOpenLoopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          ChassisSpeeds speedRobotRelative =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  // Flip so that speeds passed in are always relative to driver
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          this.drive(speedRobotRelative, true);
        });
  }

  /**
   * Stops all the modules
   *
   * @return a command stopping all the modules
   */
  public Command stop() {
    return this.runOnce(
        () -> {
          for (Module module : modules) {
            module.stop();
          }
        });
  }

  public Command translateToPose(
      Supplier<Pose2d> target,
      Supplier<ChassisSpeeds> speedsModifier,
      Constraints translationConstraints,
      Constraints headingConstraints) {
    return Commands.runOnce(
            () -> AutoAim.resetPIDControllers(getPose(), getVelocityFieldRelative()))
        .andThen(
            driveClosedLoopFieldRelative(
                    () -> {
                      return AutoAim.calculateSpeeds(
                              getPose(),
                              target.get(),
                              translationConstraints,
                              translationConstraints,
                              headingConstraints)
                          .plus(speedsModifier.get());
                    })
                .alongWith(
                    Commands.run(
                        () -> {
                          Logger.recordOutput("AutoAim/Target Pose", target.get());
                          Logger.recordOutput("AutoAim/Speeds Modifier", speedsModifier.get());
                        })));
  }

  /**
   * Autoailgns to the given pose
   *
   * <p><b>DOES NOT END</b>; Must add <code>.until()</code> with end condition
   *
   * @param target the target pose
   * @param speedsModifier Field relative speeds to be added onto the PID calculated speeds. i.e.
   *     driver requested speeds to drive along a line.
   * @return a Command driving to the target
   */
  private Command translateToPose(Supplier<Pose2d> target, Supplier<ChassisSpeeds> speedsModifier) {
    return Commands.runOnce(
            () -> AutoAim.resetPIDControllers(getPose(), getVelocityFieldRelative()))
        .andThen(
            driveClosedLoopFieldRelative(
                    () -> {
                      return AutoAim.calculateSpeeds(getPose(), target.get())
                          .plus(speedsModifier.get());
                    })
                .alongWith(
                    Commands.run(
                        () -> {
                          Logger.recordOutput("AutoAim/Target Pose", target.get());
                          Logger.recordOutput("AutoAim/Speeds Modifier", speedsModifier.get());
                        })));
  }

  /**
   * Autoaligns to the given pose
   *
   * <p><b>DOES NOT END</b>; Must add <code>.until()</code> with end condition
   *
   * @param target the target pose
   * @return a Command driving to the target
   */
  private Command translateToPose(Supplier<Pose2d> target) {
    return translateToPose(target, () -> new ChassisSpeeds());
  }

  /**
   * Autoaligns to the given pose, stopping when it's within the passed-in tolerance
   *
   * @param target the target pose
   * @param xToleranceMeters the allowed x-direction translational tolerance
   * @param yToleranceMeters the allowed y-direction translational tolerance
   * @param headingToleranceRadians the allowed heading tolerance
   * @return a Command driving to the target pose
   */
  public Command translateToPoseWithTolerance(
      Supplier<Pose2d> target,
      double xToleranceMeters,
      double yToleranceMeters,
      double headingToleranceRadians) {
    return translateToPose(target)
        .until(
            () ->
                MathUtil.isNear(target.get().getX(), getPose().getX(), xToleranceMeters)
                    && MathUtil.isNear(target.get().getY(), getPose().getY(), yToleranceMeters)
                    && MathUtil.isNear(
                        target.get().getRotation().getRadians(),
                        getPose().getRotation().getRadians(),
                        headingToleranceRadians));
  }

  private Command translateWithIntermediatePose(
      Supplier<Pose2d> target, Supplier<Pose2d> intermediate) {
    return translateToPose(intermediate)
        .until(() -> isInAutoAimTolerance(intermediate.get()))
        .andThen(translateToPose(target));
  }

  public boolean isInAutoAimTolerance(Pose2d target) {
    return isInTolerance(
        target, AutoAim.TRANSLATION_TOLERANCE_METERS, AutoAim.ROTATION_TOLERANCE_RADIANS);
  }

  public boolean isInTolerance(
      Pose2d target, double translationalToleranceMeters, double angularToleranceRadians) {
    return AutoAim.isInTolerance(
        getPose(), target, translationalToleranceMeters, angularToleranceRadians);
  }

  public Command autoAimToL1(DoubleSupplier vxModifier, DoubleSupplier vyModifier) {
    return translateToPose(
        () -> {
          Rectangle2d nearestLine = FieldUtils.L1Targets.getNearestLine(getPose());
          return new Pose2d(
              nearestLine.nearest(getPose().getTranslation()), nearestLine.getRotation());
        },
        () -> {
          ChassisSpeeds speedsModifierRobotRelative =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  vxModifier.getAsDouble(), vyModifier.getAsDouble(), 0.0, getRotation());
          // Kill all front-back requested velocity (because we want the robot to strafe)
          speedsModifierRobotRelative.vxMetersPerSecond = 0.0;
          return ChassisSpeeds.fromRobotRelativeSpeeds(speedsModifierRobotRelative, getRotation())
              .times(1.5);
        });
  }

  public boolean nearL1() {
    return isInAutoAimTolerance(
        new Pose2d(
            L1Targets.getNearestLine(getPose()).nearest(getPose().getTranslation()),
            L1Targets.getNearestLine(getPose()).getRotation()));
  }

  public boolean isNearL1Reef() {
    // TODO
    // Not sure why this is different from near l1??
    return L1Targets.getNearestLine(getPose()).getDistance(getPose().getTranslation()) < 0.3;
  }

  public boolean isNearReef(double toleranceMeters) {
    return getPose()
            .getTranslation()
            .minus(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? FieldUtils.BLUE_REEF_CENTER
                    : FieldUtils.RED_REEF_CENTER)
            .getNorm()
        < toleranceMeters;
  }

  public Command autoAimToL23(BooleanSupplier leftHanded) {
    return translateToPose(
        () -> {
          Twist2d twist = getVelocityFieldRelative().toTwist2d(0.3);
          Transform2d twistTransform =
              new Transform2d(twist.dx, twist.dy, Rotation2d.fromRadians(twist.dtheta));
          return FieldUtils.CoralTargets.getHandedClosestTargetL23(
              getPose().plus(twistTransform), leftHanded.getAsBoolean());
        });
  }

  public boolean nearL23() {
    return isInAutoAimTolerance(FieldUtils.CoralTargets.getClosestTargetL23(getPose()));
  }

  public Command autoAimToL4(BooleanSupplier leftHanded) {
    return translateToPose(
        () -> {
          Twist2d twist = getVelocityFieldRelative().toTwist2d(0.3);
          Transform2d twistTransform =
              new Transform2d(twist.dx, twist.dy, Rotation2d.fromRadians(twist.dtheta));
          return FieldUtils.CoralTargets.getHandedClosestTargetL4(
              getPose().plus(twistTransform), leftHanded.getAsBoolean());
        });
  }

  public boolean nearL4() {
    return isInAutoAimTolerance(FieldUtils.CoralTargets.getClosestTargetL4(getPose()));
  }

  // public Command autoAimToOffsetAlgaePose() {
  //   return translateToPose(
  //       () ->
  //           FieldUtils.AlgaeIntakeTargets.getOffsetLocation(
  //               FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(getPose())));
  // }

  // public boolean nearIntakeAlgaeOffsetPose() {
  //   return isInAutoAimTolerance(
  //       FieldUtils.AlgaeIntakeTargets.getOffsetLocation(
  //           FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(getPose())));
  // }

  // public Command approachAlgae() {
  //   return driveClosedLoopRobotRelative(
  //       () -> {
  //         AutoAim.resetPIDControllers(getPose(), getVelocityFieldRelative());
  //         ChassisSpeeds calculatedSpeedsRobotRelative =
  //             ChassisSpeeds.fromFieldRelativeSpeeds(
  //                 AutoAim.calculateSpeeds(
  //                     getPose(), FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(getPose())),
  //                 getRotation());
  //         if
  // (isInAutoAimTolerance(FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(getPose()))) {
  //           calculatedSpeedsRobotRelative.vyMetersPerSecond = 0.0;
  //         } else if (Robot.getScoringSide().equals(ScoringSide.LEFT)) {
  //           calculatedSpeedsRobotRelative.vyMetersPerSecond =
  //               -AutoAim.ALGAE_APPROACH_SPEED_METERS_PER_SECOND;
  //         } else if (Robot.getScoringSide().equals(ScoringSide.RIGHT)) {
  //           calculatedSpeedsRobotRelative.vyMetersPerSecond =
  //               AutoAim.ALGAE_APPROACH_SPEED_METERS_PER_SECOND;
  //         }

  //         // IDK why we have to do this but it was in reefscape so...
  //         calculatedSpeedsRobotRelative.vxMetersPerSecond *= -1;
  //         return calculatedSpeedsRobotRelative;
  //       });
  // }

  // public boolean nearAlgaeIntakePose() {
  //   return isInAutoAimTolerance(FieldUtils.AlgaeIntakeTargets.getClosestTargetPose(getPose()));
  // }

  // public Command autoAimToProcessor() {
  //   // -0.3 meters transformed needs tuning
  //   return translateWithIntermediatePose(
  //       () ->
  //           getPose()
  //               .nearest(FieldUtils.PROCESSOR_POSES)
  //               .plus(new Transform2d(0, -0.3, Rotation2d.kZero)),
  //       () -> getPose().nearest(FieldUtils.PROCESSOR_POSES));
  // }

  // public boolean nearProcessor() {
  //   return isInAutoAimTolerance(getPose().nearest(FieldUtils.PROCESSOR_POSES));
  // }

  // public Command autoAimToBarge(DoubleSupplier vyModifier) {
  //   return Commands.runOnce(
  //           () -> AutoAim.resetPIDControllers(getPose(), getVelocityFieldRelative()))
  //       .andThen(
  //           driveClosedLoopFieldRelative(
  //               () -> {
  //                 ChassisSpeeds calculatedSpeeds =
  //                     AutoAim.calculateSpeeds(
  //                         getPose(),
  //                         new Pose2d(
  //                             AutoAim.getClosestBargeXCoord(getPose()),
  //                             0.0,
  //                             AutoAim.getClosestBargeRotation(getPose())));
  //                 // Sub calculated velocity for requested velocity
  //                 calculatedSpeeds.vyMetersPerSecond = vyModifier.getAsDouble();
  //                 return calculatedSpeeds;
  //               }));
  // }

  // public boolean nearBarge() {
  //   return MathUtil.isNear(
  //           AutoAim.getClosestBargeXCoord(getPose()),
  //           getPose().getX(),
  //           AutoAim.TRANSLATION_TOLERANCE_METERS)
  //       && MathUtil.isNear(
  //           AutoAim.getClosestBargeRotation(getPose()).getRadians(),
  //           getPose().getRotation().getRadians(),
  //           AutoAim.ROTATION_TOLERANCE_RADIANS);
  // }

  /**
   * Generates a set of samples without using the async thread. Makes lots of Objects, so be careful
   * when using it irl!
   */
  private List<Samples> getSyncSamples() {
    return List.of(
        new Samples(
            Logger.getTimestamp() / 1.0e6,
            Map.of(
                new SignalID(SignalType.DRIVE, 0), modules[0].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 0), modules[0].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 1), modules[1].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 1), modules[1].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 2), modules[2].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 2), modules[2].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 3), modules[3].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 3), modules[3].getPosition().angle.getRotations(),
                new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID),
                    gyroInputs.yaw.getDegrees())));
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  /** Returns the pose estimator rotation, as returned by {@link #getPose()} */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void resetPose(Pose2d newPose) {
    estimator.resetPose(newPose);
    if (Robot.ROBOT_TYPE == RobotType.SIM) {
      swerveSimulation.setSimulationWorldPose(newPose);
      swerveSimulation.setRobotSpeeds(new ChassisSpeeds());
    }
  }

  public void setYaw(Rotation2d newYaw) {
    resetPose(new Pose2d(getPose().getTranslation(), newYaw));
  }

  @AutoLogOutput(key = "Odometry/Velocity Robot Relative")
  public ChassisSpeeds getVelocityRobotRelative() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
    return speeds;
  }

  @AutoLogOutput(key = "Odometry/Velocity Field Relative")
  public ChassisSpeeds getVelocityFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getVelocityRobotRelative(), getRotation());
  }

  public boolean isNotMoving() {
    return MathUtil.isNear(
        0,
        Math.hypot(
            getVelocityRobotRelative().vxMetersPerSecond,
            getVelocityRobotRelative().vyMetersPerSecond),
        AutoAim.VELOCITY_TOLERANCE_METERSPERSECOND);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }

  /**
   * This function bypasses the command-based framework because Choreolib handles setting
   * requirements internally. Do NOT use outside of ChoreoLib
   *
   * @return a Consumer that runs the drivebase to follow a SwerveSample with PID feedback, sample
   *     target vel feedforward, and module force feedforward.
   */
  @SuppressWarnings("resource")
  public Consumer<SwerveSample> choreoDriveController() {
    // TODO: TUNE
    final PIDController xController = new PIDController(5.0, 0.0, 0.0);
    final PIDController yController = new PIDController(5.0, 0.0, 0.0);
    final PIDController headingController = new PIDController(6.0, 0.0, 0.0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return (sample) -> {
      Pose2d pose = getPose();

      Logger.recordOutput("Choreo/Target Pose", sample.getPose());
      Logger.recordOutput("Choreo/Raw Target Speeds Field Relative", sample.getChassisSpeeds());

      ChassisSpeeds feedback =
          new ChassisSpeeds(
              xController.calculate(pose.getX(), sample.x),
              yController.calculate(pose.getY(), sample.y),
              headingController.calculate(pose.getRotation().getRadians(), sample.heading));

      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.getChassisSpeeds().plus(feedback), getPose().getRotation());
      Logger.recordOutput("Choreo/Target Speeds Robot Relative", speeds);

      this.drive(speeds, false);
    };
  }
}
