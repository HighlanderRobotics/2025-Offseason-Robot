// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.led.LEDIOReal;
import frc.robot.led.LEDSubsystem;
import frc.robot.pivot.PivotIOReal;
import frc.robot.pivot.PivotIOSim;
import frc.robot.roller.RollerIOReal;
import frc.robot.roller.RollerIOSim;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.odometry.PhoenixOdometryThread;
import frc.robot.utils.CommandXboxControllerSubsystem;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.REPLAY;
  public static final boolean TUNING_MODE = true;
  public boolean hasZeroedSinceStartup = false;

  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static enum CoralScoreTarget {
    L1(Color.kGreen),
    L2(Color.kTeal),
    L3(Color.kBlue),
    L4(LEDSubsystem.PURPLE);

    private Color color;

    private CoralScoreTarget(Color color) {
      this.color = color;
    }
  }

  public static enum CoralIntakeTarget {
    GROUND,
    STACK
  }

  //   public static enum AlgaeIntakeTarget {
  //     LOW(Color.kGreen),
  //     HIGH(Color.kTeal),
  //     STACK(Color.kBlue),
  //     GROUND(LEDSubsystem.PURPLE);

  //     private Color color;

  //     private AlgaeIntakeTarget(Color color) {
  //       this.color = color;
  //     }
  //   }

  //   public static enum AlgaeScoreTarget {
  //     BARGE(Color.kRed),
  //     PROCESSOR(Color.kYellow);

  //     private Color color;

  //     private AlgaeScoreTarget(Color color) {
  //       this.color = color;
  //     }
  //   }

  public static enum ScoringSide {
    LEFT,
    RIGHT
  }

  @AutoLogOutput private static CoralScoreTarget coralScoreTarget = CoralScoreTarget.L4;
  @AutoLogOutput private static CoralIntakeTarget coralIntakeTarget = CoralIntakeTarget.GROUND;
  //   @AutoLogOutput private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  //   @AutoLogOutput private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.BARGE;
  @AutoLogOutput private static ScoringSide scoringSide = ScoringSide.RIGHT;

  private Alert manualArmRezeroAlert;
  private Alert driverJoystickDisconnectedAlert;
  private Alert operatorJoystickDisconnectedAlert;

  private static CANBus canivore = new CANBus("*");

  private static CANBusStatus canivoreStatus = canivore.getStatus();

  // Instantiate subsystems
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());

  TalonFXConfiguration armRollerConfig =
      createRollerConfig(InvertedValue.Clockwise_Positive, 20.0, 6.62, 0.48, 0.25, 0.0);

  TalonFXConfiguration armPivotConfig =
      createPivotConfig(
              InvertedValue.Clockwise_Positive,
              ArmSubsystem.SUPPLY_CURRENT_LIMIT,
              ArmSubsystem.STATOR_CURRENT_LIMIT,
              ArmSubsystem.SENSOR_TO_MECH_RATIO,
              ArmSubsystem.KV,
              ArmSubsystem.KG,
              ArmSubsystem.KS,
              ArmSubsystem.KP,
              ArmSubsystem.KI,
              ArmSubsystem.KD)
          // this is what lets us wrap it from -180 to 180 at the bottom
          // basically think of it like the swerve turn motor
          // it's not actually fused with the cancoder, which means it shows up weird in the log
          // but that's true for the swerve as well and both seem to be fine so we'll just roll with
          // it
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

  CANcoderConfiguration armCANcoderConfig =
      createCANcoderConfig(
          SensorDirectionValue.CounterClockwise_Positive,
          ArmSubsystem.CANCODER_OFFSET,
          ArmSubsystem.CANCODER_DISCONTINUITY_POINT);

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
                  ArmSubsystem.MIN_ANGLE.getRadians(),
                  ArmSubsystem.MAX_ANGLE.getRadians(),
                  ArmSubsystem.LENGTH_METERS,
                  ArmSubsystem.MAX_VELOCITY,
                  ArmSubsystem.MAX_ACCELERATION,
                  armPivotConfig),
          new CANcoderIOReal(4, armCANcoderConfig),
          "Arm");

  TalonFXConfiguration intakeRollerConfig =
      createRollerConfig(
          InvertedValue.Clockwise_Positive,
          80.0,
          2.5,
          0.48,
          0.9,
          1); // this is for the rollers ratio

  TalonFXConfiguration intakePivotConfig =
      createPivotConfig(
          InvertedValue.CounterClockwise_Positive, 40.0, 80.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0);

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
              ? new PivotIOReal(12, IntakeSubsystem.getIntakePivotConfig())
              : new PivotIOSim(
                  IntakeSubsystem.MIN_ANGLE.getRadians(),
                  IntakeSubsystem.MAX_ANGLE.getRadians(),
                  IntakeSubsystem.LENGTH_METERS,
                  IntakeSubsystem.MAX_VELOCITY,
                  IntakeSubsystem.MAX_ACCELERATION,
                  intakePivotConfig),
          new CANrangeIOReal(0),
          new CANrangeIOReal(1),
          "Intake");

  TalonFXConfiguration climberRollerConfig =
      createRollerConfig(InvertedValue.CounterClockwise_Positive, 20.0, 5.25 / 1, 0.0, 0.0, 0.0);

  TalonFXConfiguration climberPivotConfig =
      createPivotConfig(
              InvertedValue.CounterClockwise_Positive,
              20.0,
              120.0,
              ClimberSubsystem.PIVOT_RATIO,
              ClimberSubsystem.KV,
              ClimberSubsystem.KG,
              ClimberSubsystem.KS,
              ClimberSubsystem.KP,
              ClimberSubsystem.KI,
              ClimberSubsystem.KD)
          // Disable both current limits!!!!!!!!!
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(false));

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
                  ClimberSubsystem.MIN_ANGLE.getRadians(),
                  ClimberSubsystem.MAX_ANGLE.getRadians(),
                  ClimberSubsystem.LENGTH_METERS,
                  ClimberSubsystem.MAX_VELOCITY,
                  ClimberSubsystem.MAX_ACCELERATION,
                  climberPivotConfig),
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
  private final LEDSubsystem leds = new LEDSubsystem(new LEDIOReal());

  private final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  @AutoLogOutput(key = "Superstructure/Autoaim Request")
  private Trigger autoAimReq = driver.rightBumper().or(driver.leftBumper());

  @AutoLogOutput(key = "Robot/Pre Zeroing Request")
  private Trigger preZeroingReq = driver.a();

  @AutoLogOutput(key = "Robot/Zeroing Request")
  private Trigger zeroingReq = driver.b();

  private final Superstructure superstructure =
      new Superstructure(elevator, arm, intake, climber, swerve, driver, operator);

  private final Autos autos;
  private Optional<Alliance> lastAlliance = Optional.empty();
  @AutoLogOutput boolean haveAutosGenerated = false;
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");

  // Mechanisms
  private final LoggedMechanism2d elevatorMech2d =
      new LoggedMechanism2d(3.0, Units.feetToMeters(4.0));
  private final LoggedMechanismRoot2d
      elevatorRoot = // CAD distance from origin to center of carriage at full retraction
      elevatorMech2d.getRoot(
              "Elevator", Units.inchesToMeters(5), 0.0); // now what on earth is this number
  // doesn't get updated or actually do anything it's just so i remember there's actually an
  // elevator there when i'm looking at glass
  private final LoggedMechanismLigament2d firstStage =
      new LoggedMechanismLigament2d("First Stage", Units.inchesToMeters(37), 90.0);
  private final LoggedMechanismLigament2d carriageLigament =
      new LoggedMechanismLigament2d("Carriage", 0, 90.0);
  private final LoggedMechanismLigament2d armLigament =
      new LoggedMechanismLigament2d("Arm", ArmSubsystem.LENGTH_METERS, 20.0);

  private final LoggedMechanismRoot2d intakeRoot =
      elevatorMech2d.getRoot("Intake", Units.inchesToMeters(11), 0);
  private final LoggedMechanismLigament2d intakeBase =
      new LoggedMechanismLigament2d("Intake Base", Units.inchesToMeters(9.5), 90);
  private final LoggedMechanismLigament2d intakeLigament =
      new LoggedMechanismLigament2d("Intake", IntakeSubsystem.LENGTH_METERS, 0.0);

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
    arm.setDefaultCommand(arm.setStateAngleVelocity());
    intake.setDefaultCommand(intake.setStateAngleVelocity());
    // Voltage control is intentional here
    climber.setDefaultCommand(climber.setStateAngleVoltage());

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));
    leds.setDefaultCommand(
        Commands.either(
                // enabled
                leds.setBlinkingCmd(
                    () -> getCoralScoreTarget().color,
                    () ->
                        Superstructure.getState() == SuperState.IDLE ? Color.kBlack : Color.kWhite,
                    5.0),
                // Commands.either(
                //     // if we're in an algae state, override it with the split color
                //     leds.setBlinkingSplitCmd(
                //         () -> getAlgaeIntakeTarget().color, () -> getAlgaeScoreTarget().color,
                // 5.0),
                //     // otherwise set it to the blinking pattern
                //     leds.setBlinkingCmd(
                //         () -> getCoralScoreTarget().color,
                //         () ->
                //             Superstructure.getState() == SuperState.IDLE
                //                 ? Color.kBlack
                //                 : Color.kWhite,
                //         5.0),
                //     superstructure::stateIsAlgae),
                // not enabled
                leds.setRunAlongCmd(
                    () ->
                        DriverStation.getAlliance()
                            .map((a) -> a == Alliance.Blue ? Color.kBlue : Color.kRed)
                            .orElse(Color.kWhite),
                    // () -> wrist.hasZeroed ? LEDSubsystem.PURPLE : Color.kOrange, //TODO add check
                    // for zero
                    LEDSubsystem.PURPLE,
                    4,
                    1.0),
                DriverStation::isEnabled)
            .repeatedly()
            .ignoringDisable(true));

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
                    .times(-1)
                    .times(
                        Superstructure.getState() == SuperState.PRE_CLIMB
                                || Superstructure.getState() == SuperState.CLIMB
                            ? 0.5
                            : 1.0)));

    addControllerBindings();

    autos = new Autos(swerve, arm, superstructure::resetStateForAuto);
    autoChooser.addDefaultOption("None", Commands.none());

    // Generates autos on connected
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated)
        .onTrue(Commands.print("Connected"))
        .onTrue(Commands.runOnce(this::addAutos).ignoringDisable(true));

    new Trigger(
            () -> {
              boolean allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(Commands.runOnce(this::addAutos).ignoringDisable(true));

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              System.out.println("Interrupted: " + interrupted);
              System.out.println(
                  "Interrputing: "
                      + (interrupting.isPresent() ? interrupting.get().getName() : "none"));
            });

    // Add autos on alliance change
    new Trigger(
            () -> {
              var allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(
                    leds.setBlinkingCmd(() -> Color.kWhite, () -> Color.kBlack, 20.0)
                        .withTimeout(1.0))
                .ignoringDisable(true));

    // Add autos when first connecting to DS
    new Trigger(
            () ->
                DriverStation.isDSAttached()
                    && DriverStation.getAlliance().isPresent()
                    && !haveAutosGenerated) // TODO check that the haveautosgenerated doesn't break
        // anything?
        .onTrue(Commands.print("connected"))
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(
                    leds.setBlinkingCmd(() -> Color.kWhite, () -> Color.kBlack, 20.0)
                        .withTimeout(1.0))
                .ignoringDisable(true));
    SmartDashboard.putData(
        "rezero elevator",
        elevator
            .rezero()
            .alongWith(Commands.print("dashboard rezero elevator"))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "rezero arm against cancoder",
        arm.rezeroFromEncoder()
            .alongWith(Commands.print("dashboard rezero arm against cancoder"))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "rezero arm against bumper",
        arm.rezeroAgainstRightBumper()
            .alongWith(Commands.print("dashboard rezero arm against bumper"))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Spool in climber (MANUAL STOP)",
        Commands.parallel(
            intake.setPivotVoltage(() -> -4.0),
            Commands.waitUntil(
                    () ->
                        intake.getPivotCurrentFilterValueAmps() > IntakeSubsystem.CURRENT_THRESHOLD)
                .andThen(climber.retract())));
    SmartDashboard.putData("Extend climber (MANUAL STOP)", climber.extend());
    SmartDashboard.putData("Rezero climber", climber.rezero());
    SmartDashboard.putData(
        "rezero intake",
        intake.rezero().alongWith(Commands.print("dashboard rezero intake")).ignoringDisable(true));
    SmartDashboard.putData(
        "ninety intake",
        intake.ninety().alongWith(Commands.print("dashboard ninety intake")).ignoringDisable(true));
    SmartDashboard.putData("Add autos", Commands.runOnce(this::addAutos).ignoringDisable(true));

    manualArmRezeroAlert =
        new Alert(
            "Arm has been manually rezeroed at least once this match. Arm cancoder may not be working!",
            AlertType.kWarning);

    driverJoystickDisconnectedAlert =
        new Alert("Driver controller disconnected!", AlertType.kError);
    operatorJoystickDisconnectedAlert =
        new Alert("Operator controller disconnected!", AlertType.kError);

    Logger.recordOutput(
        "test",
        new Pose2d(new Translation2d(), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(45.0))));
  }

  private TalonFXConfiguration createRollerConfig(
      InvertedValue inverted,
      double currentLimit,
      double sensorToMechanismRatio,
      double kS,
      double kV,
      double kP) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;

    config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

    return config;
  }

  private TalonFXConfiguration createPivotConfig(
      InvertedValue inverted,
      double supplyCurrentLimit,
      double statorCurrentLimit,
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

    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

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
                        .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
                .withName("Autoaim l1 Command"));

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
                        .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
                .withName("Autoaim l23"));

    // Autoaim to L4
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(() -> coralScoreTarget == CoralScoreTarget.L4)
        .whileTrue(
            Commands.parallel(
                    swerve.autoAimToL4(driver.leftBumper()),
                    Commands.waitUntil(swerve::nearL4)
                        .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
                .withName("Autoaim l4 command"));

    // Autoaim to intake algae (high, low)
    // autoAimReq
    //     .and(() -> superstructure.stateIsIntakeAlgaeReef() || superstructure.stateIsIdle())
    //     .whileTrue(
    //         Commands.parallel(
    //                 Commands.sequence(
    //                     Commands.runOnce(
    //                         () ->
    //                             Robot.setAlgaeIntakeTarget(
    //
    // AlgaeIntakeTargets.getClosestTarget(swerve.getPose()).height)),
    //                     swerve
    //                         .autoAimToOffsetAlgaePose()
    //                         .until(
    //                             new Trigger(swerve::nearIntakeAlgaeOffsetPose)
    //                                 // TODO figure out trigger order of operations? also this is
    //                                 // just
    //                                 // bad
    //                                 .and(
    //                                     () ->
    //                                         superstructure.atExtension(
    //                                             SuperState.INTAKE_ALGAE_HIGH_RIGHT))
    //                                 .or(
    //                                     () ->
    //                                         superstructure.atExtension(
    //                                             SuperState.INTAKE_ALGAE_LOW_RIGHT))),
    //                     swerve.approachAlgae()),
    //                 Commands.waitUntil(
    //                         new Trigger(swerve::nearAlgaeIntakePose)
    //                             .and(swerve::isNotMoving)
    //                             .debounce(0.08))
    //                     // .and(swerve::hasFrontTags)
    //                     .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
    //             .withName("Autoaim algae"));

    // Autoaim to processor
    // autoAimReq
    //     .and(superstructure::stateIsProcessor)
    //     .and(() -> algaeScoreTarget == AlgaeScoreTarget.PROCESSOR)
    //     .and(driver.leftBumper().negate())
    //     .whileTrue(
    //         Commands.parallel(
    //                 swerve.autoAimToProcessor(),
    //                 Commands.waitUntil(swerve::nearProcessor)
    //                     .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
    //             .withName("Autoaim algae"));

    // Autoaim to barge
    // autoAimReq
    //     .and(superstructure::stateIsBarge)
    //     .and(() -> algaeScoreTarget == AlgaeScoreTarget.BARGE)
    //     .and(driver.leftBumper().negate())
    //     .whileTrue(
    //         Commands.parallel(
    //                 swerve.autoAimToBarge(
    //                     () ->
    //                         modifyJoystick(driver.getLeftX())
    //                             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed()),
    //                 Commands.waitUntil(swerve::nearBarge)
    //                     .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy()))
    //             .withName("Autoaim barge"));

    // Operator - Set scoring/intaking levels
    operator
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L1;
                  //   algaeIntakeTarget = AlgaeIntakeTarget.GROUND;
                  //   algaeScoreTarget = AlgaeScoreTarget.PROCESSOR;
                  coralIntakeTarget = CoralIntakeTarget.GROUND;
                }));
    operator
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L2;
                  //   algaeIntakeTarget = AlgaeIntakeTarget.STACK;
                }));
    operator
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L3;
                  //   algaeIntakeTarget = AlgaeIntakeTarget.LOW;
                }));
    operator
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralScoreTarget = CoralScoreTarget.L4;
                  //   algaeIntakeTarget = AlgaeIntakeTarget.HIGH;
                  //   algaeScoreTarget = AlgaeScoreTarget.BARGE;
                }));

    operator.leftTrigger().onTrue(Commands.runOnce(() -> scoringSide = ScoringSide.LEFT));

    operator.rightTrigger().onTrue(Commands.runOnce(() -> scoringSide = ScoringSide.RIGHT));

    // Enable/disable left handed auto align
    // TODO isn't this already accounted for by the autoaim method?
    // operator.povLeft().onTrue(Commands.runOnce(() -> leftHandedTarget = true));
    // operator.povRight().onTrue(Commands.runOnce(() -> leftHandedTarget = false));

    // heading reset
    driver
        .leftStick()
        .and(driver.rightStick())
        .onTrue(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().equals(Alliance.Blue)
                            // ? Rotation2d.kCW_90deg
                            // : Rotation2d.kCCW_90deg)));
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg)));

    // ---zeroing stuff---
    // jog arm up
    operator
        .povDown()
        .and(preZeroingReq)
        .and(zeroingReq.negate())
        .whileTrue(Commands.parallel(arm.setPivotVoltage(() -> -3.0)).withTimeout(0.05));

    // jog arm down
    operator
        .povUp()
        .and(preZeroingReq)
        .and(zeroingReq.negate())
        .whileTrue(Commands.parallel(arm.setPivotVoltage(() -> 3.0)).withTimeout(0.05));

    // hold arm still when it's not being requested to jog up or down or zeroing req
    operator
        .povUp()
        .negate()
        .and(operator.povDown().negate())
        .and(preZeroingReq)
        .and(zeroingReq.negate())
        .whileTrue(arm.hold());

    // once arm is in place, run zeroing sequence
    preZeroingReq
        .and(zeroingReq)
        .whileTrue(
            Commands.sequence(
                // hold arm still while intake and elevator run zeroing concurrently
                Commands.deadline(
                    Commands.parallel(intake.runCurrentZeroing(), elevator.runCurrentZeroing()),
                    arm.hold()),
                Commands.print("done zeroing intake/elev"),
                // hold elevator and intake still while arm zeroes
                Commands.deadline(
                    arm.runCurrentZeroing(),
                    Commands.parallel(
                        elevator.setVoltage(() -> -1.0), intake.setPivotVoltage(() -> -3.0))),
                // sets exit state
                superstructure.transitionAfterZeroing(),
                // logging
                Commands.runOnce(
                    () -> {
                      Logger.recordOutput("Arm manually rezeroed", true);
                      manualArmRezeroAlert.set(true);
                    })));
    // intake.runCurrentZeroing());

    // zeroing upon startup
    // assumes cancoder hasn't failed!
    // new Trigger(() -> superstructure.stateIsIdle())
    //     .and(() -> !hasZeroedSinceStartup)
    //     .and(DriverStation::isEnabled)
    //     .onTrue(
    //         Commands.parallel(intake.runCurrentZeroing(), elevator.runCurrentZeroing())
    //             .andThen(arm.rezeroFromEncoder())
    //             .andThen(Commands.runOnce(() -> hasZeroedSinceStartup = true)));

    // Rezero arm against cancoder
    driver.x().onTrue(Commands.runOnce(() -> arm.rezeroFromEncoder()).ignoringDisable(true));

    // antijam algae
    // i am pulling these numbers out of my ass
    // put down intake and put arm horizontal
    operator
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                intake.setPivotVoltage(() -> -3.0),
                arm.setPivotAngle(() -> Rotation2d.fromDegrees(-90.0)),
                Commands.waitUntil(
                        () ->
                            intake.isNearAngle(IntakeSubsystem.ZEROING_ANGLE)
                                && arm.isNearAngle(Rotation2d.fromDegrees(-90.0)))
                    .andThen(elevator.setExtensionMeters(() -> Units.inchesToMeters(30)))));

    // eject coral
    operator
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                arm.setRollerVelocity(() -> -14.0), intake.setRollerVelocity(() -> -20.0)));

    // Force the robot to think it doesn't have a coral
    // probably should not do this
    operator.leftStick().onTrue(Commands.runOnce(() -> arm.hasCoral = false));

    new Trigger(() -> DriverStation.isJoystickConnected(0))
        .negate()
        .onTrue(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> driverJoystickDisconnectedAlert.set(false)));

    new Trigger(() -> DriverStation.isJoystickConnected(1))
        .negate()
        .onTrue(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(true)))
        .onFalse(Commands.runOnce(() -> operatorJoystickDisconnectedAlert.set(false)));
  }

  private void addAutos() {
    System.out.println("------- Regenerating Autos");
    System.out.println(
        "Regenerating Autos on " + DriverStation.getAlliance().map((a) -> a.toString()));

    autoChooser.addOption("Left stack auto", autos.getLeftStackAuto());
    autoChooser.addOption("Right stack auto", autos.getRightStackAuto());
    autoChooser.addOption("Center auto", autos.getAlgaeAuto());
    autoChooser.addOption(
        "left taxi",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().equals(Alliance.Blue)
                            ? Rotation2d.k180deg.plus(Rotation2d.fromDegrees(-45.0))
                            : Rotation2d.kZero.plus(Rotation2d.fromDegrees(-45.0)))),
            swerve
                .driveClosedLoopRobotRelative(() -> new ChassisSpeeds(1.0, 0.0, 0.0))
                .withTimeout(2)));
    autoChooser.addOption(
        "right taxi",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    swerve.setYaw(
                        DriverStation.getAlliance().equals(Alliance.Blue)
                            ? Rotation2d.kZero.plus(Rotation2d.fromDegrees(45.0))
                            : Rotation2d.k180deg.plus(Rotation2d.fromDegrees(45.0)))),
            swerve
                .driveClosedLoopRobotRelative(() -> new ChassisSpeeds(1.0, 0.0, 0.0))
                .withTimeout(2)));
    haveAutosGenerated = true;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.periodic();

    Logger.recordOutput(
        "Mechanism Poses",
        // new Pose3d[] {
        //   new Pose3d( // first stage
        //       new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
        //   // carriage
        //   new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d()),
        //   new Pose3d( // arm
        //           Translation3d.kZero, new Rotation3d(0, arm.getPivotAngle().getRadians(), 0.0))
        //       .transformBy(
        //           new Transform3d(
        //               new Translation3d(
        //                   ArmSubsystem.VERTICAL_OFFSET_METERS
        //                       * Math.cos(Math.PI / 2 - arm.getPivotAngle().getRadians()),
        //                   0,
        //                   elevator.getExtensionMeters()
        //                   // - ArmSubsystem.VERTICAL_OFFSET_METERS
        //                   //     * Math.sin(Math.PI / 2 - arm.getPivotAngle().getRadians())),
        //                   ),
        //               Rotation3d.kZero)),
        //   new Pose3d( // intake
        //       new Translation3d(0, 0, 0), // Units.inchesToMeters(10.265)
        //       // new Rotation3d(Math.PI, intake.getPivotAngle().getRadians(), Math.PI))
        //       new Rotation3d(intake.getPivotAngle().getRadians(), 0, 0))
        // });
        new Pose3d[] {
          new Pose3d(
              // first stage
              new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
          // carriage
          new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d()),
          Pose3d.kZero,
          //   Pose3d.kZero
          new Pose3d( // intake
                  new Translation3d(0, 0, 0),
                  // Units.inchesToMeters(10.265)
                  // new Rotation3d(Math.PI, intake.getPivotAngle().getRadians(), Math.PI))
                  new Rotation3d(intake.getPivotAngle().getRadians(), 0, 0))
              .transformBy(
                  new Transform3d(
                      new Translation3d(
                          0,
                          0,
                          1
                              * IntakeSubsystem.VERTICAL_OFFSET_METERS
                              * Math.cos(intake.getPivotAngle().getRadians() / 2.0)),
                      Rotation3d.kZero))
        });

    carriageLigament.setLength(elevator.getExtensionMeters());
    // Minus 90 to make it relative to horizontal
    armLigament.setAngle(arm.getPivotAngle().getDegrees() - 90);
    intakeLigament.setAngle(intake.getPivotAngle());

    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Mechanism/Elevator", elevatorMech2d);
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

  //   public static void setAlgaeIntakeTarget(AlgaeIntakeTarget target) {
  //     algaeIntakeTarget = target;
  //   }

  //   public static AlgaeIntakeTarget getAlgaeIntakeTarget() {
  //     return algaeIntakeTarget;
  //   }

  //   public static void setAlgaeScoreTarget(AlgaeScoreTarget target) {
  //     algaeScoreTarget = target;
  //   }

  //   public static AlgaeScoreTarget getAlgaeScoreTarget() {
  //     return algaeScoreTarget;
  //   }

  public static ScoringSide getScoringSide() {
    return scoringSide;
  }

  public static void setScoringSide(ScoringSide newSide) {
    scoringSide = newSide;
  }
}
