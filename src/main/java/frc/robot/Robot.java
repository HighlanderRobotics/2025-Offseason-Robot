// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.ArmIOReal;
import frc.robot.arm.ArmIOSim;
import frc.robot.arm.ArmSubsystem;
// import frc.robot.shoulder.ShoulderSubsystem;
// import frc.robot.intake.IntakeSubsystem;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorIOSim;
import frc.robot.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.LoggedRobot;

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

  private final ArmSubsystem arm =
      new ArmSubsystem(ROBOT_TYPE != RobotType.SIM ? new ArmIOReal() : new ArmIOSim());
  // private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  // private final IntakeSubsystem intake = new IntakeSubsystem();

  private final Superstructure superstructure = new Superstructure(elevator, arm);

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
