package frc.robot.swerve;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.constants.SwerveConstants;
import frc.robot.swerve.gyro.GyroIO;
import frc.robot.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.swerve.module.Module;
import frc.robot.swerve.module.ModuleIOReal;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveConstants swerveConstants;

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private SwerveDriveKinematics kinematics;

  private SwerveDrivePoseEstimator estimator;

  private SwerveModulePosition[] lastModulePositions;
  private Rotation2d rawGyroRotation;
  private Rotation2d lastGyroRotation;

  public SwerveSubsystem(SwerveConstants swerveConstants, GyroIO gyroIO) {
    // TODO: MAKE THESE WORK FOR SIM AS WELL
    modules =
        new Module[] {
          new Module(new ModuleIOReal(swerveConstants.getFrontLeftModule())),
          new Module(new ModuleIOReal(swerveConstants.getFrontRightModule())),
          new Module(new ModuleIOReal(swerveConstants.getBackLeftModule())),
          new Module(new ModuleIOReal(swerveConstants.getBackRightModule()))
        };

    this.gyroIO = gyroIO;

    lastModulePositions = new SwerveModulePosition[modules.length];
  }

  @Override
  public void periodic() {

    // Updates each module
    for (Module module : modules) {
      module.periodic();
    }

    gyroIO.updateInputs(gyroInputs);
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
      // Update the twist with the gyro
      twist = new Twist2d(twist.dx, twist.dy, rawGyroRotation.minus(lastGyroRotation).getRadians());
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
      gyroInputs.roll.getRadians(),
      gyroInputs.pitch.getRadians(),
      gyroInputs.yaw.getRadians()
    );
  }

  // TODO: RESET POSE IN SIM ONCE IMPLEMENTED
  public void resetPose(Pose2d newPose) {
    estimator.resetPose(newPose);
  }

  @AutoLogOutput(key = "Odometry/Velocity Robot Relative")
  public ChassisSpeeds getVelocityRobotRelative() {
    ChassisSpeeds speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules)
                .map((module) -> module.getState())
                .toArray(SwerveModuleState[]::new));
    return speeds;
  }

  @AutoLogOutput(key = "Odometry/Velocity Field Relative")
  public ChassisSpeeds getVelocityFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getVelocityRobotRelative(), getRotation());
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }
}
