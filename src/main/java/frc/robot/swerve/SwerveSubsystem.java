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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.swerve.constants.SwerveConstants;
import frc.robot.swerve.gyro.GyroIO;
import frc.robot.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.swerve.module.Module;
import frc.robot.swerve.module.ModuleIOReal;
import frc.robot.swerve.module.ModuleIOSim;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveConstants swerveConstants;

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

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
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final Optional<SwerveDriveSimulation> swerveSimulation;

  public SwerveSubsystem(
      SwerveConstants swerveConstants,
      GyroIO gyroIO,
      Optional<SwerveDriveSimulation> swerveSimulation) {
    this.swerveConstants = swerveConstants;
    if (Robot.ROBOT_TYPE == RobotType.SIM && swerveSimulation.isPresent()) {
      // Add simulated modules
      modules =
          new Module[] {
            new Module(
                new ModuleIOSim(
                    swerveConstants.getFrontLeftModule(), swerveSimulation.get().getModules()[0])),
            new Module(
                new ModuleIOSim(
                    swerveConstants.getFrontRightModule(), swerveSimulation.get().getModules()[1])),
            new Module(
                new ModuleIOSim(
                    swerveConstants.getBackLeftModule(), swerveSimulation.get().getModules()[2])),
            new Module(
                new ModuleIOSim(
                    swerveConstants.getBackRightModule(), swerveSimulation.get().getModules()[3]))
          };
    } else {
      // Add real modules
      modules =
          new Module[] {
            new Module(new ModuleIOReal(swerveConstants.getFrontLeftModule())),
            new Module(new ModuleIOReal(swerveConstants.getFrontRightModule())),
            new Module(new ModuleIOReal(swerveConstants.getBackLeftModule())),
            new Module(new ModuleIOReal(swerveConstants.getBackRightModule()))
          };
    }

    this.gyroIO = gyroIO;

    this.swerveSimulation = swerveSimulation;

    this.kinematics = new SwerveDriveKinematics(swerveConstants.getModuleTranslations());
    // Std devs copied from reefscape
    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.6, 0.6, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.4));
  }

  @Override
  public void periodic() {

    // Updates each module
    for (Module module : modules) {
      module.periodic();
    }

    gyroIO.updateInputs(gyroInputs);

    updateOdometry();
  }

  private void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Converts time continuous chassis speeds to setpoints after the specified time (dtSeconds)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert drivetrain setpoint into individual module setpoints
    final SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];

    for (int i = 0; i < optimizedStates.length; i++) {
      if (openLoop) {
        optimizedStates[i] = modules[i].runOpenLoop(states[i], true);
      } else {
        optimizedStates[i] = modules[i].runClosedLoop(states[i]);
      }
    }
  }

  // TODO: THIS WILL ALL NEED UPDATING AFTER ODO THREAD IMPL
  private void updateOdometry() {

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];

    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      double dist = modules[moduleIndex].getPositionMeters();
      Rotation2d rot = modules[moduleIndex].getAngle();

      modulePositions[moduleIndex] = new SwerveModulePosition(dist, rot);

      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              dist - lastModulePositions[moduleIndex].distanceMeters,
              rot.minus(lastModulePositions[moduleIndex].angle));
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Expresses the movement of the robot since last update in x, y, and theta
    // Doesn't use gyro data
    Twist2d twist = kinematics.toTwist2d(moduleDeltas);

    // No gyro data
    if (!gyroInputs.isConnected) {
      // If the gyro doesn't update, use module positions
      rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
    } else {
      rawGyroRotation = gyroInputs.yaw;
    }

    lastGyroRotation = rawGyroRotation;
    // Update!
    estimator.update(rawGyroRotation, modulePositions);
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
}
