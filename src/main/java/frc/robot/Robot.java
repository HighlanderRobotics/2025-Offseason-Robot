// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.ArmSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.pivot.PivotIOReal;
import frc.robot.pivot.PivotIOSim;
import frc.robot.pivot.PivotWithCANcoderIOReal;
import frc.robot.roller.RollerIOReal;
import frc.robot.roller.RollerIOSim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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

  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(
          ROBOT_TYPE != RobotType.SIM ? new ElevatorIOReal() : new ElevatorIOSim());

  // TODO: fill in correct values for these subsystems

  private TalonFXConfiguration rollerConfig(double currentLimit) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  private TalonFXConfiguration pivotConfig(
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
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

  private final ArmSubsystem arm =
      new ArmSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(14, rollerConfig(20.0))
              : new RollerIOSim(
                  0.01,
                  1.0,
                  new SimpleMotorFeedforward(0.0, 0.24),
                  new ProfiledPIDController(
                      0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 1))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotWithCANcoderIOReal(
                  15, 16, pivotConfig(20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0))
              : new PivotIOSim((44.0 / 16.0) * 23, 0.0, 180.0, 23.0),
          "Arm");

  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(17, rollerConfig(20.0))
              : new RollerIOSim(
                  0.01,
                  1.0,
                  new SimpleMotorFeedforward(0.0, 0.24),
                  new ProfiledPIDController(
                      0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 1))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotIOReal(18, pivotConfig(20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0))
              : new PivotIOSim((44.0 / 16.0) * 23, 0.0, 90.0, 15),
          "Intake");

  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          ROBOT_TYPE != RobotType.SIM
              ? new RollerIOReal(20, rollerConfig(20.0))
              : new RollerIOSim(
                  0.01,
                  1.0,
                  new SimpleMotorFeedforward(0.0, 0.24),
                  new ProfiledPIDController(
                      0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 1))),
          ROBOT_TYPE != RobotType.SIM
              ? new PivotIOReal(21, pivotConfig(20.0, 10, 1.0, 0.4, 0.2, 0.5, 0.0, 0.0))
              : new PivotIOSim((44.0 / 16.0) * 23, 0.0, 90.0, 9.25),
          "Climber");

  private final Superstructure superstructure = new Superstructure(elevator, arm, intake);

  public Robot() {
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
}
