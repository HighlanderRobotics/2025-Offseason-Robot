package frc.robot.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.swerve.constants.KelpieSwerveConstants;
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
import frc.robot.util.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  public static final SwerveConstants SWERVE_CONSTANTS = new KelpieSwerveConstants();

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
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

  private static final SignalID GYRO_SIGNAL_ID = new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID);
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

  private final Optional<SwerveDriveSimulation> swerveSimulation;

  private Alert usingSyncOdoAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);

  public SwerveSubsystem(Optional<SwerveDriveSimulation> swerveSimulation) {
    if (Robot.ROBOT_TYPE == RobotType.SIM && swerveSimulation.isPresent()) {
      // Add simulated modules
      modules =
          new Module[] {
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontLeftModule(), swerveSimulation.get().getModules()[0])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontRightModule(),
                    swerveSimulation.get().getModules()[1])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackLeftModule(), swerveSimulation.get().getModules()[2])),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackRightModule(), swerveSimulation.get().getModules()[3]))
          };
    } else {
      // Add real modules
      modules =
          new Module[] {
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontLeftModule())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontRightModule())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackLeftModule())),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackRightModule()))
          };
    }

    this.gyroIO = Robot.ROBOT_TYPE != RobotType.SIM ? new GyroIOReal(SWERVE_CONSTANTS.getGyroID()) : new GyroIOSim(swerveSimulation.get().getGyroSimulation());

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

          Tracer.trace("Update odometry", this::updateOdometry);
        });
  }

  /**
   * Runs the modules to the specified ChassisSpeeds (robot velocity)
   * @param speeds the ChassisSpeeds to run the drivetrain at
   * @param openLoop boolean for if the drivetrain should run with feedforward control (open loop) or with feedback control (closed loop)
   */
  private void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Converts time continuous chassis speeds to setpoints after the specified time (dtSeconds)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert drivetrain setpoint into individual module setpoints
    final SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    // Makes sure each wheel isn't asked to go above its max. Recalcs the states if needed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVE_CONSTANTS.getMaxLinearSpeed());
    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("SwerveStates/Setpoints", states);

    if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Swerve/Target Speeds", speeds);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];

    for (int i = 0; i < optimizedStates.length; i++) {
      if (openLoop) {
        // Heuristic to enable/disable FOC
        // enables FOC if the robot is moving at 90% of drivetrain max speed
        final boolean focEnable =
            Math.sqrt(
                    Math.pow(this.getVelocityRobotRelative().vxMetersPerSecond, 2)
                        + Math.pow(this.getVelocityRobotRelative().vyMetersPerSecond, 2))
                < SWERVE_CONSTANTS.getMaxLinearSpeed()
                    * 0.9; // 0.9 is 90% of drivetrain max speed
        optimizedStates[i] = modules[i].runOpenLoop(states[i], focEnable);
      } else {
        optimizedStates[i] = modules[i].runClosedLoop(states[i]);
      }
    }

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
  }

  private void updateOdometry() {

    List<Samples> sampledStates = odometryThreadInputs.sampledStates;
    // Use sync samples if there aren't any async ones
    if (sampledStates.size() == 0 || Robot.isSimulation() || sampledStates.get(0).values().isEmpty()) {
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
      if (!gyroInputs.isConnected || sample.values().get(GYRO_SIGNAL_ID) == null) {
        hasNullGyroRotation = true;
      } else {
        hasNullGyroRotation = false;
      }

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

  public Rotation3d getRotation3d() {
    return new Rotation3d(
        gyroInputs.roll.getRadians(), gyroInputs.pitch.getRadians(), gyroInputs.yaw.getRadians());
  }

  public void resetPose(Pose2d newPose) {
    estimator.resetPose(newPose);
    if (swerveSimulation.isPresent()) {
      swerveSimulation.get().setSimulationWorldPose(newPose);
      swerveSimulation.get().setRobotSpeeds(new ChassisSpeeds());
    }
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

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }

  /**
   * Drive closed-loop at robot relative speeds
   *
   * @param speeds robot relative speed setpoint
   * @return a command driving to target speeds
   */
  public Command driveClosedLoop(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(getVelocityFieldRelative(), false));
  }

  /**
   * Drive at a robot-relative speed open-loop.
   *
   * @param speeds the robot-relative speed setpoint.
   * @return a Command driving to the target speeds.
   */
  public Command driveOpenLoop(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), true));
  }

  /**
   * Drive closed-loop at a field relative speed
   *
   * @param speeds the field-relative speed setpoint
   * @return a Command driving to those speeds
   */
  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return driveClosedLoop(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Drives open-loop. Speeds field relative to driver.
   *
   * @param speeds the field-relative speeds to drive at
   * @return a Command driving at those speeds
   */
  public Command driveTeleop(Supplier<ChassisSpeeds> speeds) {
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

  public Command stop() {
    return this.run(() -> {
      for (Module module : modules) {
        module.stop();
      }
    });
  }
}
