// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmIOReal;
import frc.robot.arm.ArmIOSim;
import frc.robot.arm.ArmSubsystem;
import frc.robot.beambreak.BeambreakIOReal;
import frc.robot.climb.ClimberIOReal;
import frc.robot.climb.ClimberSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeIOReal;
import frc.robot.intake.IntakeIOSim;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.roller.RollerIOReal;
import frc.robot.roller.RollerIOSim;
import frc.robot.routing.RoutingSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  // Various field locations
  public static enum ReefTarget {
    L1(0.0, SuperState.L1),
    L2(0.0, SuperState.L2),
    L3(0.0, SuperState.L3),
    L4(0.0, SuperState.L4);

    public final double outtakeSpeed; // TODO got no clue
    public final SuperState state;

    private ReefTarget(double outtakeSpeed, SuperState state) {
      this.outtakeSpeed = outtakeSpeed;
      this.state = state;
    }
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

  // Current score/intake targets
  @AutoLogOutput private static ReefTarget coralTarget = ReefTarget.L4;
  @AutoLogOutput private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  @AutoLogOutput private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.BARGE;

  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  // ---define triggers---
  @AutoLogOutput
  public static Trigger preScoreReq =
      new Trigger(() -> SmartDashboard.getBoolean("prescorereq", false));

  @AutoLogOutput
  public static Trigger scoreReq = new Trigger(() -> SmartDashboard.getBoolean("scorereq", false));

  @AutoLogOutput
  public static Trigger intakeAlgaeReq =
      new Trigger(() -> SmartDashboard.getBoolean("intake algae req", false));

  @AutoLogOutput
  public static Trigger intakeCoralReq =
      new Trigger(() -> SmartDashboard.getBoolean("intakereq", false));

  // ---instantiate subsystems---
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());
  private final ArmSubsystem arm =
      new ArmSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ArmIOReal() : new ArmIOSim(),
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(new TalonFXConfiguration(), false, 9)
              : new RollerIOSim(0.01, 16.0 / 64.0),
          new BeambreakIOReal(0, false));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new IntakeIOReal() : new IntakeIOSim(),
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(new TalonFXConfiguration(), false, 13)
              : new RollerIOSim(0.01, 0.2)); // idk but idc
  private final RoutingSubsystem routing =
      new RoutingSubsystem(
          new RollerIOReal(new TalonFXConfiguration(), false, 14, 15),
          new BeambreakIOReal(1, false)); // TODO ids
  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          new RollerIOReal(new TalonFXConfiguration(), false, 17), new ClimberIOReal());

  // ---instantiate superstructure---
  private final Superstructure superstructure =
      new Superstructure(elevator, arm, intake, routing, climber);

  // ---instantiate mechanism sims---
  private final LoggedMechanism2d elevatorMech2d =
      new LoggedMechanism2d(3.0, Units.feetToMeters(6.0));
  private final LoggedMechanismRoot2d elevatorRoot =
      elevatorMech2d.getRoot(
          "Elevator",
          Units.inchesToMeters(21.5 + 20),
          0.0); // CAD distance from origin to center of carriage at full retraction
  private final LoggedMechanismLigament2d carriageLigament =
      new LoggedMechanismLigament2d("Carriage", 0, 90);
  private final LoggedMechanismLigament2d armLigament =
      new LoggedMechanismLigament2d("Arm", Units.inchesToMeters(15.7), 120);
  private final LoggedMechanismRoot2d intakeRoot =
      elevatorMech2d.getRoot("Intake Root", Units.inchesToMeters(2 + 20), 0.0);
  private final LoggedMechanismLigament2d intakeBaseLigament =
      new LoggedMechanismLigament2d("Intake Base", 0, 90);
  private final LoggedMechanismLigament2d intakePivotLigament =
      new LoggedMechanismLigament2d("Intake Pivot", Units.inchesToMeters(18), 90);

  @SuppressWarnings("resource")
  public Robot() {
    // ---Set up logging as per AdvantageKit docs---
    Logger.recordMetadata("Codebase", "Offseason 2025"); // Set a metadata value
    switch (ROBOT_TYPE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
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

    // ---Set default commands for each subsystem---
    elevator.setDefaultCommand(
        elevator.setStateExtension()); // TODO not sure if this stuff needs to be hold?
    arm.setDefaultCommand(arm.setStateAngleVoltage());
    routing.setDefaultCommand(routing.setStateRollerVoltage());
    intake.setDefaultCommand(intake.setStateAngleVoltage());

    // ---SmartDashboard bindings---
    SmartDashboard.putData("idle", superstructure.changeStateTo(() -> SuperState.IDLE));
    SmartDashboard.putData("l1", superstructure.changeStateTo(() -> SuperState.L1));
    SmartDashboard.putData("l2", superstructure.changeStateTo(() -> SuperState.L2));

    // surely there's a better way to do this but i'm sort of a clown and also lazy
    SmartDashboard.putData(
        "toggle intakereq",
        Commands.runOnce(
            () ->
                SmartDashboard.putBoolean(
                    "intakereq", !SmartDashboard.getBoolean("intakereq", false))));
    SmartDashboard.putData(
        "toggle scorereq",
        Commands.runOnce(
            () ->
                SmartDashboard.putBoolean(
                    "scorereq", !SmartDashboard.getBoolean("scorereq", false))));

    SmartDashboard.putData(
        "toggle pre scorereq",
        Commands.runOnce(
            () ->
                SmartDashboard.putBoolean(
                    "prescorereq", !SmartDashboard.getBoolean("prescorereq", false))));
    SmartDashboard.putData(
        "toggle arm bb", Commands.runOnce(() -> arm.setSimBeambreak(!arm.hasPiece())));
    SmartDashboard.putData(
        "toggle intake bb", Commands.runOnce(() -> intake.setSimBeambreak(!intake.hasCoral())));
    SmartDashboard.putData(
        "toggle intake algae req",
        Commands.runOnce(
            () ->
                SmartDashboard.putBoolean(
                    "intake algae req", !SmartDashboard.getBoolean("intake algae req", false))));
    SmartDashboard.putData("L1", Commands.runOnce(() -> coralTarget = ReefTarget.L1));
    SmartDashboard.putData("L2", Commands.runOnce(() -> coralTarget = ReefTarget.L2));
    SmartDashboard.putData("L3", Commands.runOnce(() -> coralTarget = ReefTarget.L3));
    SmartDashboard.putData("L4", Commands.runOnce(() -> coralTarget = ReefTarget.L4));

    SmartDashboard.putData(
        "high", Commands.runOnce(() -> algaeIntakeTarget = AlgaeIntakeTarget.HIGH));
    SmartDashboard.putData(
        "low", Commands.runOnce(() -> algaeIntakeTarget = AlgaeIntakeTarget.LOW));
    SmartDashboard.putData(
        "ground", Commands.runOnce(() -> algaeIntakeTarget = AlgaeIntakeTarget.GROUND));

    SmartDashboard.putData(
        "barge", Commands.runOnce(() -> algaeScoreTarget = AlgaeScoreTarget.BARGE));
    SmartDashboard.putData(
        "processor", Commands.runOnce(() -> algaeScoreTarget = AlgaeScoreTarget.PROCESSOR));

    // ---add sim mechanisms---
    elevatorRoot.append(carriageLigament);
    carriageLigament.append(armLigament);
    intakeRoot.append(intakeBaseLigament);
    intakeBaseLigament.append(intakePivotLigament);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Logger.recordOutput(
        "Mechanism Poses",
        new Pose3d[] {
          new Pose3d( // first stage
              new Translation3d(0, 0, elevator.getExtensionMeters() / 2.0), new Rotation3d()),
          // carriage
          new Pose3d(new Translation3d(0, 0, elevator.getExtensionMeters()), new Rotation3d())
        });
    carriageLigament.setLength(elevator.getExtensionMeters());
    armLigament.setAngle(arm.getAngle().getDegrees());
    armLigament.setColor(new Color8Bit(Color.kPurple));
    intakeBaseLigament.setLength(Units.inchesToMeters(11.3));
    intakePivotLigament.setColor(new Color8Bit(Color.kBlue));
    intakePivotLigament.setAngle(intake.getAngle().getDegrees());
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Mechanism/Elevator", elevatorMech2d);

    superstructure.periodic();
  }

  public static ReefTarget getCoralTarget() {
    return coralTarget;
  }

  public static AlgaeIntakeTarget getAlgaeIntakeTarget() {
    return algaeIntakeTarget;
  }

  public static AlgaeScoreTarget getAlgaeScoreTarget() {
    return algaeScoreTarget;
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
}
