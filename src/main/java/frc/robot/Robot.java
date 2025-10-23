// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.cancoder.CANcoderIOReal;
import frc.robot.canrange.CANrangeIOReal;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.pivot.PivotIOReal;
import frc.robot.pivot.PivotIOSim;
import frc.robot.roller.RollerIOReal;
import frc.robot.roller.RollerIOSim;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.odometry.PhoenixOdometryThread;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils.AlgaeIntakeTargets;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  // TODO add tuning mode switch

  public static enum CoralScoreTarget {
    L1,
    L2,
    L3,
    L4;
  }

  public static enum CoralIntakeTarget {
    GROUND,
    STACK
  }

  public static enum AlgaeIntakeTarget {
    LOW,
    HIGH,
    STACK,
    GROUND
  }

  public static enum AlgaeScoreTarget {
    BARGE,
    PROCESSOR
  }

  public static enum ScoringSide {
    LEFT,
    RIGHT
  }

  @AutoLogOutput private static CoralScoreTarget coralScoreTarget = CoralScoreTarget.L4;
  @AutoLogOutput private static CoralIntakeTarget coralIntakeTarget = CoralIntakeTarget.GROUND;
  @AutoLogOutput private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  @AutoLogOutput private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.BARGE;
  @AutoLogOutput private static ScoringSide scoringSide = ScoringSide.RIGHT;

  private static CANBus canivore = new CANBus("*");

  private static CANBusStatus canivoreStatus = canivore.getStatus();

  // Instantiate subsystems
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());

  // TODO tune these config values
  TalonFXConfiguration armRollerConfig =
      createRollerConfig(InvertedValue.CounterClockwise_Positive, 20.0);
  TalonFXConfiguration armPivotConfig =
      createPivotConfig(
          InvertedValue.CounterClockwise_Positive, 20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0);
  CANcoderConfiguration armCANcoderConfig =
      createCANcoderConfig(SensorDirectionValue.Clockwise_Positive, 0.0, 0.0);

  TalonFXConfiguration intakeRollerConfig =
      createRollerConfig(InvertedValue.CounterClockwise_Positive, 20.0);
  TalonFXConfiguration intakePivotConfig =
      createPivotConfig(
          InvertedValue.CounterClockwise_Positive, 20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0);

  TalonFXConfiguration climberRollerConfig =
      createRollerConfig(InvertedValue.CounterClockwise_Positive, 20.0);
  TalonFXConfiguration climberPivotConfig =
      createPivotConfig(
          InvertedValue.CounterClockwise_Positive, 20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0);

  // TODO tuning sim values espicall for pivot sims
  private final ArmSubsystem arm =
      new ArmSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(8, armRollerConfig)
              : new RollerIOSim(
                  ArmSubsystem.jKgMetersSquared,
                  ArmSubsystem.PIVOT_RATIO,
                  new SimpleMotorFeedforward(ArmSubsystem.KS, ArmSubsystem.KV),
                  new ProfiledPIDController(
                      ArmSubsystem.KP,
                      ArmSubsystem.KI,
                      ArmSubsystem.KD,
                      new TrapezoidProfile.Constraints(
                          ArmSubsystem.MAX_VELOCITY, ArmSubsystem.MAX_ACCELERATION))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotIOReal(9, armPivotConfig)
              : new PivotIOSim(
                  ArmSubsystem.PIVOT_RATIO,
                  ArmSubsystem.MIN_ANGLE.getRadians(),
                  ArmSubsystem.MAX_ANGLE.getRadians(),
                  ArmSubsystem.LENGTH_METERS,
                  ArmSubsystem.KP,
                  ArmSubsystem.KI,
                  ArmSubsystem.KD,
                  ArmSubsystem.KI,
                  ArmSubsystem.KG,
                  ArmSubsystem.KV,
                  ArmSubsystem.MAX_VELOCITY,
                  ArmSubsystem.MAX_ACCELERATION),
          new CANcoderIOReal(4, armCANcoderConfig),
          "Arm");

  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(13, intakeRollerConfig)
              : new RollerIOSim(
                  IntakeSubsystem.jKgMetersSquared,
                  IntakeSubsystem.PIVOT_RATIO,
                  new SimpleMotorFeedforward(IntakeSubsystem.KS, IntakeSubsystem.KV),
                  new ProfiledPIDController(
                      IntakeSubsystem.KP,
                      IntakeSubsystem.KI,
                      IntakeSubsystem.KD,
                      new TrapezoidProfile.Constraints(
                          IntakeSubsystem.MAX_VELOCITY, IntakeSubsystem.MAX_ACCELERATION))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotIOReal(12, intakePivotConfig)
              : new PivotIOSim(
                  IntakeSubsystem.PIVOT_RATIO,
                  IntakeSubsystem.MIN_ANGLE.getRadians(),
                  IntakeSubsystem.MAX_ANGLE.getRadians(),
                  IntakeSubsystem.LENGTH_METERS,
                  IntakeSubsystem.KP,
                  IntakeSubsystem.KI,
                  IntakeSubsystem.KD,
                  IntakeSubsystem.KI,
                  IntakeSubsystem.KG,
                  IntakeSubsystem.KV,
                  IntakeSubsystem.MAX_VELOCITY,
                  IntakeSubsystem.MAX_ACCELERATION),
          new CANrangeIOReal(0),
          new CANrangeIOReal(1),
          "Intake");

  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(15, climberRollerConfig)
              : new RollerIOSim(
                  ClimberSubsystem.jKgMetersSquared,
                  ClimberSubsystem.PIVOT_RATIO,
                  new SimpleMotorFeedforward(ClimberSubsystem.KS, ClimberSubsystem.KV),
                  new ProfiledPIDController(
                      ClimberSubsystem.KP,
                      ClimberSubsystem.KI,
                      ClimberSubsystem.KD,
                      new TrapezoidProfile.Constraints(
                          ClimberSubsystem.MAX_VELOCITY, ClimberSubsystem.MAX_ACCELERATION))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotIOReal(14, climberPivotConfig)
              : new PivotIOSim(
                  ClimberSubsystem.PIVOT_RATIO,
                  ClimberSubsystem.MIN_ANGLE.getRadians(),
                  ClimberSubsystem.MAX_ANGLE.getRadians(),
                  ClimberSubsystem.LENGTH_METERS,
                  ClimberSubsystem.KP,
                  ClimberSubsystem.KI,
                  ClimberSubsystem.KD,
                  ClimberSubsystem.KI,
                  ClimberSubsystem.KG,
                  ClimberSubsystem.KV,
                  ClimberSubsystem.MAX_VELOCITY,
                  ClimberSubsystem.MAX_ACCELERATION),
          "Climber");

  // Maple Sim Stuff
  private final DriveTrainSimulationConfig driveTrainSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          // TODO: MAKE SURE THIS MODULE IS CORRECT
          .withSwerveModule(
              COTS.ofMark4n(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  // Still not sure where the 1.5 came from
                  1.5,
                  // Running l2+ swerve modules
                  2))
          .withTrackLengthTrackWidth(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthX()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthY()))
          .withBumperSize(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperWidth()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperLength()))
          .withRobotMass(SwerveSubsystem.SWERVE_CONSTANTS.getMass());

  private final SwerveDriveSimulation swerveSimulation =
      new SwerveDriveSimulation(driveTrainSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
  // Subsystem initialization
  private final SwerveSubsystem swerve = new SwerveSubsystem(swerveSimulation);

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  @AutoLogOutput(key = "Superstructure/Autoaim Request")
  private Trigger autoAimReq = driver.rightBumper().or(driver.leftBumper());

  // TODO impl autoaiming left vs right

  private final Superstructure superstructure =
      new Superstructure(elevator, arm, intake, climber, swerve, driver, operator);

  private final Autos autos;
  private Optional<Alliance> lastAlliance = Optional.empty();
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  @SuppressWarnings("resource")
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "2025 Offseason");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_TYPE.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (ROBOT_TYPE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    Logger.recordOutput("Canivore Status", canivoreStatus.Status);

    PhoenixOdometryThread.getInstance().start();

    // Set default commands
    elevator.setDefaultCommand(elevator.setStateExtension());
    arm.setDefaultCommand(arm.setStateAngleVoltage());
    intake.setDefaultCommand(intake.setStateAngleVoltage());
    climber.setDefaultCommand(climber.setStateAngleVoltage());

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));

    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation);
    }

    swerve.setDefaultCommand(
        swerve.driveOpenLoopFieldRelative(
            () ->
                new ChassisSpeeds(
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getRightX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                    .times(-1)));

    addControllerBindings();

    autos = new Autos(swerve, arm);
    // autoChooser.addDefaultOption("None", autos.getNoneAuto());
    // TODO add autos trigger
  }

  private TalonFXConfiguration createRollerConfig(InvertedValue inverted, double currentLimit) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  private TalonFXConfiguration createPivotConfig(
      InvertedValue inverted,
      double currentLimit,
      double sensorToMechRatio,
      double kV,
      double kG,
      double kS,
      double kP,
      double kI,
      double kD) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = sensorToMechRatio;

    return config;
  }

  private CANcoderConfiguration createCANcoderConfig(
      SensorDirectionValue directionValue,
      double MagnetOffset,
      double AbsoluteSensorDiscontinuityPoint) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = directionValue;
    config.MagnetSensor.MagnetOffset = MagnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint;

    return config;
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  private void addControllerBindings() {
    // Autoaim to L1
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(() -> coralScoreTarget == CoralScoreTarget.L1)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToL1(
                    () ->
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                    () ->
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()),
                Commands.waitUntil(swerve::nearL1)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to L2/3
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(
            () ->
                coralScoreTarget == CoralScoreTarget.L2 || coralScoreTarget == CoralScoreTarget.L3)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToL23(driver.leftBumper()),
                Commands.waitUntil(swerve::nearL23)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to L4
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(() -> coralScoreTarget == CoralScoreTarget.L4)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToL4(driver.leftBumper()),
                Commands.waitUntil(swerve::nearL4)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to intake algae (high, low)
    autoAimReq
        .and(superstructure::stateIsIntakeAlgaeReef)
        .or(superstructure::stateIsIdle)
        .whileTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Robot.setAlgaeIntakeTarget(
                                AlgaeIntakeTargets.getClosestTarget(swerve.getPose()).height)),
                    swerve
                        .autoAimToOffsetAlgaePose()
                        .until(
                            new Trigger(swerve::nearIntakeAlgaeOffsetPose)
                                // TODO figure out trigger order of operations? also this is just
                                // bad
                                .and(
                                    () ->
                                        superstructure.atExtension(
                                            SuperState.INTAKE_ALGAE_HIGH_RIGHT))
                                .or(
                                    () ->
                                        superstructure.atExtension(
                                            SuperState.INTAKE_ALGAE_LOW_RIGHT))),
                    swerve.approachAlgae()),
                Commands.waitUntil(
                        new Trigger(swerve::nearAlgaeIntakePose)
                            .and(swerve::isNotMoving)
                            .debounce(0.08))
                    // .and(swerve::hasFrontTags)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to processor
    autoAimReq
        .and(superstructure::stateIsProcessor)
        .and(() -> algaeScoreTarget == AlgaeScoreTarget.PROCESSOR)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToProcessor(),
                Commands.waitUntil(swerve::nearProcessor)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to barge
    autoAimReq
        .and(superstructure::stateIsBarge)
        .and(() -> algaeScoreTarget == AlgaeScoreTarget.BARGE)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToBarge(
                    () ->
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()),
                Commands.waitUntil(swerve::nearBarge)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Operator - Set scoring/intaking levels
    operator
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L1;
                  algaeIntakeTarget = AlgaeIntakeTarget.GROUND;
                  algaeScoreTarget = AlgaeScoreTarget.PROCESSOR;
                }));
    operator
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L2;
                  algaeIntakeTarget = AlgaeIntakeTarget.STACK;
                }));
    operator
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L3;
                  algaeIntakeTarget = AlgaeIntakeTarget.LOW;
                }));
    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L4;
                  algaeIntakeTarget = AlgaeIntakeTarget.HIGH;
                  algaeScoreTarget = AlgaeScoreTarget.BARGE;
                }));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> scoringSide = ScoringSide.LEFT));

    operator.rightTrigger().onTrue(Commands.runOnce(() -> scoringSide = ScoringSide.RIGHT));

    // Enable/disable left handed auto align
    // TODO isn't this already accounted for by the autoaim method?
    // operator.povLeft().onTrue(Commands.runOnce(() -> leftHandedTarget = true));
    // operator.povRight().onTrue(Commands.runOnce(() -> leftHandedTarget = false));

    // heading reset
    // driver
    //     .leftStick()
    //     .and(driver.rightStick())
    //     .onTrue(
    //         Commands.runOnce(
    //             () ->
    //                 swerve.setYaw(
    //                     DriverStation.getAlliance().equals(Alliance.Blue)
    //                         ? Rotation2d.kZero
    //                         : Rotation2d.k180deg)));
  }

  private void addAutos() {
    System.out.println("------- Regenerating Autos");
    System.out.println(
        "Regenerating Autos on " + DriverStation.getAlliance().map((a) -> a.toString()));
    // autoChooser.addOption("Triangle Test", autos.getTestTriangle());
    // autoChooser.addOption("Sprint Test", autos.getTestSprint());
    // autoChooser.addOption("LM to H", autos.LMtoH());
    // autoChooser.addOption("RM to G", autos.RMtoG());
    // autoChooser.addOption("4.5 L Outside", autos.LOtoJ());
    // autoChooser.addOption("4.5 R Outside", autos.ROtoE());
    // autoChooser.addOption("4.5 L Inside", autos.LItoK());
    // autoChooser.addOption("4.5 R Inside", autos.RItoD());
    // autoChooser.addOption("Push Auto", autos.PMtoPL());
    // autoChooser.addOption("Algae auto", autos.CMtoGH());
    // autoChooser.addOption("!!! DO NOT RUN!! 2910 auto", autos.LOtoA());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.periodic();

    Logger.recordOutput(
        "Mechanism Poses",
        new Pose3d[] {
          // First stage
          new Pose3d(0, 0, elevator.getExtensionMeters() / 2, Rotation3d.kZero),
          // Carriage
          new Pose3d(0, 0, elevator.getExtensionMeters(), Rotation3d.kZero),
          // Arm
          Pose3d.kZero,
          // Intake
          Pose3d.kZero
        });
  }

  @Override
  public void simulationInit() {
    // Sets the odometry pose to start at the same place as maple sim pose
    swerve.resetPose(swerveSimulation.getSimulatedDriveTrainPose());
  }

  @Override
  public void simulationPeriodic() {
    // Update maple simulation
    SimulatedArena.getInstance().simulationPeriodic();
    // Log simulated pose
    Logger.recordOutput("MapleSim/Pose", swerveSimulation.getSimulatedDriveTrainPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static void setCoralScoreTarget(CoralScoreTarget target) {
    coralScoreTarget = target;
  }

  public static CoralScoreTarget getCoralScoreTarget() {
    return coralScoreTarget;
  }

  public static void setCoralIntakeTarget(CoralIntakeTarget target) {
    coralIntakeTarget = target;
  }

  public static CoralIntakeTarget getCoralIntakeTarget() {
    return coralIntakeTarget;
  }

  public static void setAlgaeIntakeTarget(AlgaeIntakeTarget target) {
    algaeIntakeTarget = target;
  }

  public static AlgaeIntakeTarget getAlgaeIntakeTarget() {
    return algaeIntakeTarget;
  }

  public static void setAlgaeScoreTarget(AlgaeScoreTarget target) {
    algaeScoreTarget = target;
  }

  public static AlgaeScoreTarget getAlgaeScoreTarget() {
    return algaeScoreTarget;
  }

  public static ScoringSide getScoringSide() {
    return scoringSide;
  }
}
