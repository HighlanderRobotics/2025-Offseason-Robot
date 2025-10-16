// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmIOReal;
import frc.robot.arm.ArmIOSim;
import frc.robot.arm.ArmSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.led.LEDIOReal;
import frc.robot.led.LEDSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.FieldUtils.AlgaeIntakeTargets;
import java.util.Optional;
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

  public static enum AlgaeIntakeTarget {
    LOW(Color.kGreen),
    HIGH(Color.kTeal),
    STACK(Color.kBlue),
    GROUND(LEDSubsystem.PURPLE);

    private Color color;

    private AlgaeIntakeTarget(Color color) {
      this.color = color;
    }
  }

  public static enum AlgaeScoreTarget {
    BARGE(Color.kRed),
    PROCESSOR(Color.kYellow);

    private Color color;

    private AlgaeScoreTarget(Color color) {
      this.color = color;
    }
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

  @AutoLogOutput private boolean haveAutosGenerated = false;

  // Instantiate subsystems
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());

  private final ArmSubsystem arm =
      new ArmSubsystem(ROBOT_TYPE != RobotType.SIM ? new ArmIOReal() : new ArmIOSim());

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final LEDSubsystem leds = new LEDSubsystem(new LEDIOReal());

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

    // Set default commands
    elevator.setDefaultCommand(elevator.setStateExtension());
    arm.setDefaultCommand(arm.setStateAngleVoltage());
    intake.setDefaultCommand(intake.setStateAngleVoltage());
    climber.setDefaultCommand(climber.setStateAngleVoltage());

    driver.setDefaultCommand(driver.rumbleCmd(0.0, 0.0));
    operator.setDefaultCommand(operator.rumbleCmd(0.0, 0.0));
    leds.setDefaultCommand(
        Commands.either(
                // enabled
                Commands.either(
                    // if we're in an algae state, override it with the split color
                    leds.setBlinkingSplitCmd(
                        getAlgaeIntakeTarget().color, getAlgaeScoreTarget().color, 5.0),
                    // otherwise set it to the blinking pattern
                    leds.setBlinkingCmd(
                        getCoralScoreTarget().color,
                        superstructure.getState() == SuperState.IDLE ? Color.kBlack : Color.kWhite,
                        5.0),
                    superstructure::stateIsAlgae),
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

    addControllerBindings();

    autos = new Autos(swerve, arm);
    // autoChooser.addDefaultOption("None", autos.getNoneAuto());
    // TODO add autos trigger

    // Add autos on alliance change
    new Trigger(
            () -> {
              var allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(
            Commands.runOnce(() -> addAutos())
                .alongWith(leds.setBlinkingCmd(Color.kWhite, Color.kBlack, 20.0).withTimeout(1.0))
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
                .alongWith(leds.setBlinkingCmd(Color.kWhite, Color.kBlack, 20.0).withTimeout(1.0))
                .ignoringDisable(true));
  }

  private void addControllerBindings() {
    // Autoaim to L1
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(() -> coralScoreTarget == CoralScoreTarget.L1)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToL1(),
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
                swerve.autoAimToL23(),
                Commands.waitUntil(swerve::nearL23)
                    .andThen(driver.rumbleCmd(1.0, 1.0).withTimeout(0.75).asProxy())));

    // Autoaim to L4
    autoAimReq
        .and(superstructure::stateIsCoral)
        .and(() -> coralScoreTarget == CoralScoreTarget.L4)
        .whileTrue(
            Commands.parallel(
                swerve.autoAimToL4(),
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
                        .autoAimToOffsetAlgae()
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
                swerve.autoAimToBarge(),
                Commands.waitUntil(swerve::isNearBarge)
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
    haveAutosGenerated = true;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.periodic();
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
