// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmIOReal;
import frc.robot.arm.ArmSubsystem;
import frc.robot.beambreak.BeambreakIOReal;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeIOReal;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.roller.RollerIOReal;
import frc.robot.routing.RoutingSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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

  public static Trigger preScoreReq =
      new Trigger(() -> true); // TODO this would be the driver button
  public static Trigger scoreReq = new Trigger(() -> true);
  public static Trigger intakeAlgaeReq = new Trigger(() -> true);
  public static Trigger intakeCoralReq = new Trigger(() -> true);

  private final ElevatorSubsystem elevator = new ElevatorSubsystem(new ElevatorIOReal());
  private final ArmSubsystem arm =
      new ArmSubsystem(
          new ArmIOReal(),
          new RollerIOReal(
              new int[] {1},
              new TalonFXConfiguration(),
              false), // TODO so this is just embarrassing
          new BeambreakIOReal(0, false));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          new IntakeIOReal(), new RollerIOReal(new int[] {0}, new TalonFXConfiguration(), false));
  private final RoutingSubsystem routing =
      new RoutingSubsystem(
          new RollerIOReal(new int[] {1, 2}, new TalonFXConfiguration(), false),
          new BeambreakIOReal(0, false)); // TODO ids

  private final Superstructure superstructure = new Superstructure(elevator, arm, intake);

  public Robot() {
    // Set up logging as per AdvantageKit docs
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    // Set default commands for each subsystem
    elevator.setDefaultCommand(
        elevator.setStateExtension()); // TODO not sure if this stuff needs to be hold?
    arm.setDefaultCommand(arm.setStateAngleVoltage());
    routing.setDefaultCommand(routing.setStateRollerVoltage());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
