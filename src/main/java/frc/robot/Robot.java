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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmIOReal;
import frc.robot.arm.ArmSubsystem;
import frc.robot.beambreak.BeambreakIOReal;
import frc.robot.climb.ClimberIOReal;
import frc.robot.climb.ClimberSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeIOReal;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.roller.RollerIOReal;
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
    NET,
    PROCESSOR
  }

  // Current score/intake targets
  @AutoLogOutput private static ReefTarget currentCoralTarget = ReefTarget.L4;
  @AutoLogOutput private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  @AutoLogOutput private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.NET;

  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  // ---define triggers---
  public static Trigger preScoreReq =
      new Trigger(() -> true); // TODO this would be the driver button
  public static Trigger scoreReq = new Trigger(() -> true);
  public static Trigger intakeAlgaeReq = new Trigger(() -> true);
  public static Trigger intakeCoralReq = new Trigger(() -> true);

  // ---instantiate subsystems---
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());
  private final ArmSubsystem arm =
      new ArmSubsystem(
          new ArmIOReal(),
          new RollerIOReal(new TalonFXConfiguration(), false, 9),
          new BeambreakIOReal(0, false));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          new IntakeIOReal(), new RollerIOReal(new TalonFXConfiguration(), false, 13));
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
      new LoggedMechanism2d(3.0, Units.feetToMeters(4.0));
  private final LoggedMechanismRoot2d elevatorRoot =
      elevatorMech2d.getRoot(
          "Elevator",
          Units.inchesToMeters(21.5),
          0.0); // CAD distance from origin to center of carriage at full retraction
  private final LoggedMechanismLigament2d carriageLigament =
      new LoggedMechanismLigament2d("Carriage", 0, 90);

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
    SmartDashboard.putData("elevator sim test", elevator.setExtension(() -> 1));

    // ---add sim mechanisms---
    elevatorRoot.append(carriageLigament);
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
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Mechanism/Elevator", elevatorMech2d);

    superstructure.periodic();
  }

  public static ReefTarget getCurrentCoralTarget() {
    return currentCoralTarget;
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
