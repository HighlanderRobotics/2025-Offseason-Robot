package frc.robot.swerve.constants;

import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import frc.robot.camera.Camera.CameraConstants;
import frc.robot.swerve.module.Module.ModuleConstants;

public class OffseasonBotSwerveConstants extends SwerveConstants {

  @Override
  public String getName() {
    return "Offseason Bot";
  }

  // TODO explain +0.5
  @Override
  public ModuleConstants getFrontLeftModuleConstants() {
    return new ModuleConstants(0, "Front Left", 0, 1, 0, Rotation2d.fromRotations(-0.351 + 0.5));
  }

  @Override
  public ModuleConstants getFrontRightModuleConstants() {
    return new ModuleConstants(1, "Front Right", 2, 3, 1, Rotation2d.fromRotations(-0.380859375));
  }

  @Override
  public ModuleConstants getBackLeftModuleConstants() {
    return new ModuleConstants(2, "Back Left", 4, 5, 2, Rotation2d.fromRotations(0.469 + 0.5));
  }

  @Override
  public ModuleConstants getBackRightModuleConstants() {
    return new ModuleConstants(3, "Back Right", 6, 7, 3, Rotation2d.fromRotations(-0.069));
  }

  @Override
  public int getGyroID() {
    return 0;
  }

  public Pigeon2Configuration getGyroConfig() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPosePitch = -0.002175945555791259;
    config.MountPose.MountPoseRoll = 0.120527483522892;
    config.MountPose.MountPoseYaw = -89.22984313964844;
    return config;
  }

  @Override
  public TalonFXConfiguration getTurnConfiguration(int cancoderID) {
    var turnConfig = new TalonFXConfiguration();
    // Current limits
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Inverts
    turnConfig.MotorOutput.Inverted =
        getTurnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Fused Cancoder
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    turnConfig.Feedback.RotorToSensorRatio = getTurnGearRatio();
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackRotorOffset = 0.0;
    // Controls Gains
    // Copied from Kelpie
    turnConfig.Slot0.kV = 0.42962962963; // ((5800 / 60) / getTurnGearRatio()) / 12
    turnConfig.Slot0.kA = 0.031543;
    turnConfig.Slot0.kS = 0.27;
    turnConfig.Slot0.kP = 20.0;
    turnConfig.Slot0.kD = 0.68275;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / getTurnGearRatio();
    turnConfig.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (getTurnGearRatio() * 0.005);
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return turnConfig;
  }

  @Override
  public TalonFXConfiguration getDriveConfiguration() {
    var driveConfig = new TalonFXConfiguration();
    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Inverts
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Sensor
    // Meters per second
    driveConfig.Feedback.SensorToMechanismRatio = getDriveRotorToMeters();
    // Current control gains
    // Gains copied from Kelpie Swerve Constants
    // May need tuning
    driveConfig.Slot0.kV = 5.0;
    // kT (stall torque / stall current) converted to linear wheel frame
    driveConfig.Slot0.kA = 0.0; // (9.37 / 483.0) / getDriveRotorToMeters(); // 3.07135116146;
    driveConfig.Slot0.kS = 10.0;
    driveConfig.Slot0.kP = 300.0;
    driveConfig.Slot0.kD = 0.0; // 1.0;

    driveConfig.TorqueCurrent.TorqueNeutralDeadband = 10.0;

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = getMaxLinearSpeed();
    driveConfig.MotionMagic.MotionMagicAcceleration = getMaxLinearAcceleration();

    return driveConfig;
  }

  @Override
  public CANcoderConfiguration getCancoderConfiguration(Rotation2d cancoderOffset) {
    final var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        getTurnMotorInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    return cancoderConfig;
  }

  @Override
  public double getBumperLength() {
    return Units.inchesToMeters(36.6);
  }

  @Override
  public double getBumperWidth() {
    return Units.inchesToMeters(36.6);
  }

  @Override
  public double getDriveGearRatio() {
    // For SDS Mk4i L2 swerve
    return 6.75;
  }

  @Override
  public double getTurnGearRatio() {
    // For Mk4i swerve
    return 150.0 / 7.0;
  }

  @Override
  public Mass getMass() {
    // 115.67 lb is weight of robot + battery. 28.95 lb is weight of bumpers
    return Pound.of(115.76 + 28.95);
  }

  @Override
  public double getMaxLinearAcceleration() {
    // 9.0 Nm is slightly less than x60 stall torque (w) (9.37 Nm)
    // a = F/m
    // F = w/d
    // d = distance torque is applied to
    // Does this make sense??
    // It comes out to like 3 m/s^2 (max speed in 1.5 seconds roughly)
    // Kelpie's was 14 but it seemed arbitrary...
    // return ((9.37 * getDriveGearRatio()) / getDriveRotorToMeters()) / getMass().in(Kilogram);
    return 14.0;
  }

  @Override
  public double getMaxLinearSpeed() {
    // For SDS Mk4i L2 swerve with Kraken x60 FOC for drive
    return Units.feetToMeters(15.0);
  }

  @Override
  public double getTrackWidthX() {
    return Units.inchesToMeters(23.729);
  }

  @Override
  public double getTrackWidthY() {
    return Units.inchesToMeters(23.729);
  }

  @Override
  public CameraConstants[] getCameraConstants() {
    final Matrix<N3, N3> RIGHT_ELEVATOR_CAMERA_INTRINSICS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            913.3459639243549,
            0.0,
            654.21164902256,
            0.0,
            911.6840291055018,
            390.9894635553962,
            0.0,
            0.0,
            1.0);
    // not sure if java is ok with scientific notation
    final Matrix<N8, N1> RIGHT_ELEVATOR_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.05105666053513881,
            -0.0851020439698792,
            1.6403586037918962E-4,
            1.4631274866318736E-4,
            0.026868333033731307,
            -0.0020587869606236604,
            0.004088029419887703,
            -9.803976647692152E-4);
    final Matrix<N3, N3> RIGHT_DRIVEBASE_CAMERA_INTRINSICS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            913.7374772109177,
            0.0,
            649.9523505968294,
            0.0,
            912.2179388005287,
            408.3423836218782,
            0.0,
            0.0,
            1.0);
    final Matrix<N8, N1> RIGHT_DRIVEBASE_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.05036490156210035,
            -0.07581268532820391,
            -0.0013038584793272766,
            -6.07909967325906E-4,
            0.01921938959039581,
            -0.0027324349808302245,
            0.006026186907998818,
            4.3831144132513674E-4);
    final Matrix<N3, N3> LEFT_ELEVATOR_CAMERA_INTRINSICS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            919.5001918618356,
            0.0,
            828.0140509099408,
            0.0,
            919.8653650079931,
            703.8904313760787,
            0.0,
            0.0,
            1.0);
    final Matrix<N8, N1> LEFT_ELEVATOR_DIST_COEFFS =
        MatBuilder.fill(Nat.N8(), Nat.N1(), 0.026, -0.048, 0, 0, 0.073, -0.024, 0.031, 0.053);
    final Matrix<N3, N3> LEFT_DRIVEBASE_CAMERA_INTRINSICS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            910.4543686924517,
            0.0,
            636.0626170515789,
            0.0,
            909.9744395853056,
            382.6154028749968,
            0.0,
            0.0,
            1.0);
    final Matrix<N8, N1> LEFT_DRIVEBASE_DIST_COEFFS =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            0.040119422316274016,
            -0.04835252969283134,
            -9.763490623926644E-4,
            -0.0013527552120382006,
            -0.005821189294822927,
            -9.426338337651241E-4,
            0.006453303380916122,
            0.003459079233022941);

    final CameraConstants rightElevatorCamConstants =
        new CameraConstants(
            "Right_Elevator",
            new Transform3d(
                new Translation3d(-0.139, -0.211, 0.798),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(135 + 90 + 180),
                    Units.degreesToRadians(90 + 180))),
            RIGHT_ELEVATOR_CAMERA_INTRINSICS,
            RIGHT_ELEVATOR_DIST_COEFFS);
    final CameraConstants rightDrivebaseCamConstants =
        new CameraConstants(
            "Right_Drivebase",
            new Transform3d(
                new Translation3d(-0.287, -0.299, 0.227),
                new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(-30))),
            RIGHT_DRIVEBASE_CAMERA_INTRINSICS,
            RIGHT_DRIVEBASE_DIST_COEFFS);
    final CameraConstants leftElevatorCamConstants =
        new CameraConstants(
            "Left_Elevator",
            new Transform3d(
                new Translation3d(-0.139, 0.211, 0.798),
                new Rotation3d(
                    0, Units.degreesToRadians(135 + 90 + 180), Units.degreesToRadians(90))),
            LEFT_ELEVATOR_CAMERA_INTRINSICS,
            LEFT_ELEVATOR_DIST_COEFFS);
    final CameraConstants leftDrivebaseCamConstants =
        new CameraConstants(
            "Left_Drivebase",
            new Transform3d(
                new Translation3d(-0.1, 0.33, 0.15),
                new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(47))),
            LEFT_DRIVEBASE_CAMERA_INTRINSICS,
            LEFT_DRIVEBASE_DIST_COEFFS);
    return new CameraConstants[] {
      rightElevatorCamConstants,
      rightDrivebaseCamConstants,
      leftElevatorCamConstants,
      leftDrivebaseCamConstants
    };
  }

  // TODO
  @Override
  public double getHeadingVelocityKP() {
    // Copied from Alpha
    return 6.0;
  }
}
