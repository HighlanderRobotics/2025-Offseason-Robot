// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.gyro.GyroIOReal;
import frc.robot.swerve.gyro.GyroIOSim;
import frc.robot.swerve.odometry.PhoenixOdometryThread;
import frc.robot.util.CommandXBoxControllerSubsystem;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
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
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final Superstructure superstructure = new Superstructure(elevator, shoulder, intake);

  // Maple Sim Stuff
  private final DriveTrainSimulationConfig driveTrainSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          // TODO: MAKE SURE THIS MODULE IS CORRECT
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  SwerveSubsystem.SWERVE_CONSTANTS.getDriveGearRatio(),
                  SwerveSubsystem.SWERVE_CONSTANTS.getTurnGearRatio(),
                  // These friction voltages are copied from Reefscape repo
                  Volts.of(0.1),
                  Volts.of(0.2),
                  Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getWheelRadiusMeters()),
                  // Copied from Reefscape
                  KilogramSquareMeters.of(0.03),
                  // Copied from Reefscape
                  1.5))
          .withTrackLengthTrackWidth(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthX()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthY()))
          .withBumperSize(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperWidth()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperLength()));

  private final Optional<SwerveDriveSimulation> swerveSimulation =
      ROBOT_TYPE == RobotType.SIM
          ? Optional.of(
              new SwerveDriveSimulation(driveTrainSimConfig, new Pose2d(3, 3, Rotation2d.kZero)))
          : Optional.empty();

  // Subsystem initialization
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(swerveSimulation);

  private final CommandXBoxControllerSubsystem driver = new CommandXBoxControllerSubsystem(0);
  private final CommandXBoxControllerSubsystem operator = new CommandXBoxControllerSubsystem(1);

  public Robot() {
    // Init Logger
    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "Comp2025");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_TYPE.toString());

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

    PhoenixOdometryThread.getInstance().start();

    if (ROBOT_TYPE == RobotType.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation.get());
    }

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getRightX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                    .times(-1)));
  }

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationInit() {
    // Sets the odometry pose to start at the same place as maple sim pose
    swerve.resetPose(swerveSimulation.get().getSimulatedDriveTrainPose());
  }

  @Override
  public void simulationPeriodic() {
    // Update maple simulation
    SimulatedArena.getInstance().simulationPeriodic();
    // Log simulated pose
    Logger.recordOutput("MapleSim/Pose", swerveSimulation.get().getSimulatedDriveTrainPose());
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
