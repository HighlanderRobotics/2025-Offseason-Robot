// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.CoralIntakeTarget;
import frc.robot.Robot.CoralScoreTarget;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.ArmSubsystem.ArmState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystem.ClimberState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.IntakeSubsystem.IntakeState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. Hopefully we can get named
   * positions set up in cad to make this easier?
   */
  public enum SuperState {
    IDLE(ElevatorState.IDLE, ArmState.IDLE, IntakeState.IDLE),

    INTAKE_CORAL_GROUND(ElevatorState.IDLE, ArmState.IDLE, IntakeState.INTAKE_CORAL),

    PRE_HANDOFF(ElevatorState.IDLE, ArmState.IDLE, IntakeState.PRE_HANDOFF),
    HANDOFF(ElevatorState.IDLE, ArmState.HANDOFF, IntakeState.HANDOFF),

    INTAKE_CORAL_STACK(
        ElevatorState.INTAKE_CORAL_STACK, ArmState.INTAKE_CORAL_STACK, IntakeState.CLIMB),

    READY_CORAL(ElevatorState.IDLE, ArmState.READY_CORAL, IntakeState.IDLE),

    PRE_L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.PRE_L1),
    L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.SCORE_L1),
    PRE_L2(ElevatorState.PRE_L2, ArmState.PRE_L2, IntakeState.IDLE),
    SCORE_L2(ElevatorState.L2, ArmState.SCORE_L2, IntakeState.IDLE),
    PRE_L3(ElevatorState.PRE_L3, ArmState.PRE_L3, IntakeState.IDLE),
    SCORE_L3(ElevatorState.L3, ArmState.SCORE_L3, IntakeState.IDLE),
    PRE_L4(ElevatorState.PRE_L4, ArmState.PRE_L4, IntakeState.IDLE),
    SCORE_L4(ElevatorState.L4, ArmState.SCORE_L4, IntakeState.IDLE),

    INTAKE_ALGAE_HIGH(
        ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF, IntakeState.IDLE),
    INTAKE_ALGAE_LOW(
        ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF, IntakeState.IDLE),
    INTAKE_ALGAE_STACK(
        ElevatorState.INTAKE_ALGAE_STACK, ArmState.INTAKE_ALGAE_STACK, IntakeState.IDLE),
    INTAKE_ALGAE_GROUND(
        ElevatorState.INTAKE_ALGAE_GROUND, ArmState.INTAKE_ALGAE_GROUND, IntakeState.IDLE),

    READY_ALGAE(ElevatorState.READY_ALGAE, ArmState.READY_ALGAE, IntakeState.IDLE),

    PRE_BARGE(ElevatorState.BARGE, ArmState.PRE_BARGE, IntakeState.IDLE),
    SCORE_BARGE(ElevatorState.BARGE, ArmState.SCORE_BARGE, IntakeState.IDLE),

    PRE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.PRE_PROCESSOR, IntakeState.IDLE),
    SCORE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.SCORE_PROCESSOR, IntakeState.IDLE),

    PRE_CLIMB(
        ElevatorState.PRE_CLIMB, ArmState.PRE_CLIMB, IntakeState.CLIMB, ClimberState.PRE_CLIMB),
    CLIMB(ElevatorState.CLIMB, ArmState.CLIMB, IntakeState.CLIMB, ClimberState.CLIMB);

    public final ElevatorState elevatorState;
    public final ArmState armState;
    public final IntakeState intakeState;
    public final ClimberState climberState;

    private SuperState(
        ElevatorState elevatorState,
        ArmState armState,
        IntakeState intakeState,
        ClimberState climberState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.climberState = climberState;
    }

    private SuperState(ElevatorState elevatorState, ArmState armState, IntakeState intakeState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.climberState = ClimberState.IDLE;
    }

    public boolean isCoral() {
      return this == INTAKE_CORAL_GROUND
          || this == PRE_HANDOFF
          || this == HANDOFF
          || this == INTAKE_CORAL_STACK
          || this == READY_CORAL
          || this == PRE_L1
          || this == L1
          || this == PRE_L2
          || this == SCORE_L2
          || this == PRE_L3
          || this == SCORE_L3
          || this == PRE_L4
          || this == SCORE_L4;
    }

    public boolean isAlgae() {
      return this == INTAKE_ALGAE_HIGH
          || this == INTAKE_ALGAE_LOW
          || this == INTAKE_ALGAE_STACK
          || this == INTAKE_ALGAE_GROUND
          || this == READY_ALGAE
          || this == PRE_BARGE
          || this == SCORE_BARGE
          || this == PRE_PROCESSOR
          || this == SCORE_PROCESSOR;
    }
  }

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;
  private final ClimberSubsystem climber;
  private final SwerveSubsystem swerve;
  private final CommandXboxControllerSubsystem driver;
  private final CommandXboxControllerSubsystem operator;

  // Declare triggers
  @AutoLogOutput(key = "Superstructure/Pre Score Request")
  public Trigger preScoreReq;

  @AutoLogOutput(key = "Superstructure/Score Request")
  public Trigger scoreReq;

  @AutoLogOutput(key = "Superstructure/Coral Intake Request")
  public Trigger intakeCoralReq;

  @AutoLogOutput(key = "Superstructure/Algae Intake Request")
  public Trigger intakeAlgaeReq;

  @AutoLogOutput(key = "Superstructure/Pre Climb Request")
  public Trigger preClimbReq;

  @AutoLogOutput(key = "Superstructure/Climb Confirm Request")
  public Trigger climbConfReq;

  @AutoLogOutput(key = "Superstructure/Climb Cancel Request")
  public Trigger climbCancelReq;

  /** Creates a new Superstructure. */
  public Superstructure(
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      IntakeSubsystem intake,
      ClimberSubsystem climber,
      SwerveSubsystem swerve,
      CommandXboxControllerSubsystem driver,
      CommandXboxControllerSubsystem operator) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.climber = climber;
    this.swerve = swerve;
    this.driver = driver;
    this.operator = operator;

    addTriggers();
    addTransitions();

    stateTimer.start();
  }

  private void addTriggers() {
    preScoreReq = driver.rightTrigger().or(Autos.autoPreScoreReq.and(DriverStation::isAutonomous));

    scoreReq =
        driver.rightTrigger().negate().or(Autos.autoScoreReq.and(DriverStation::isAutonomous));

    intakeCoralReq =
        driver.leftTrigger().or(Autos.autoIntakeCoralReq.and(DriverStation::isAutonomous));

    intakeAlgaeReq =
        driver.leftBumper().or(Autos.autoIntakeAlgaeReq.and(DriverStation::isAutonomous));

    // TODO seems sus
    preClimbReq =
        driver
            .x()
            .and(driver.pov(-1).negate())
            .debounce(0.25)
            .or(operator.x().and(operator.pov(-1).negate()).debounce(0.5));

    climbConfReq = driver.rightTrigger();

    climbCancelReq =
        driver
            .y()
            .debounce(0.5)
            .or(operator.leftStick().and(operator.rightTrigger()).debounce(0.5));
  }

  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
    Logger.recordOutput("Superstructure/State Timer", stateTimer.get());
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger) {
    // maps triggers to the transitions
    trigger.and(new Trigger(() -> state == start)).onTrue(changeStateTo(end));
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd some command to run while making the transition
   */
  private void bindTransition(SuperState start, SuperState end, Trigger trigger, Command cmd) {
    // maps triggers to the transitions
    trigger
        .and(new Trigger(() -> state == start))
        .onTrue(Commands.parallel(changeStateTo(end), cmd));
  }

  public boolean atExtension(SuperState state) {
    return elevator.atExtension(state.elevatorState.getExtensionMeters())
        && arm.isNearAngle(state.armState.getAngle())
        && intake.isNearAngle(state.intakeState.getAngle());
  }

  public boolean atExtension() {
    return atExtension(state);
  }

  private Command changeStateTo(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state from " + state + " to " + nextState);
              stateTimer.reset();
              this.prevState = this.state;
              this.state = nextState;
              setSubstates();
            })
        .ignoringDisable(true);
  }

  public Command changeStateTo(Supplier<SuperState> state) {
    return changeStateTo(state.get());
  }

  private void setSubstates() {
    elevator.setState(state.elevatorState);
    arm.setState(state.armState);
    intake.setState(state.intakeState);
    climber.setState(state.climberState);
  }

  private void addTransitions() {
    // ---Intake coral ground---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_CORAL_GROUND,
        intakeCoralReq.and(() -> Robot.getCoralIntakeTarget() == CoralIntakeTarget.GROUND));

    bindTransition(
        SuperState.INTAKE_CORAL_GROUND,
        SuperState.PRE_HANDOFF,
        new Trigger(intake::hasCoral).debounce(0.1));

    bindTransition(
        SuperState.PRE_HANDOFF,
        SuperState.HANDOFF,
        // TODO maybe make the hascorals and stuff triggers inside intake?
        new Trigger(intake::hasCoral).debounce(0.1).and(this::atExtension));

    bindTransition(
        SuperState.HANDOFF,
        SuperState.READY_CORAL,
        new Trigger(arm::hasCoral)
            .debounce(0.1)
            .and(intake::hasCoral)
            .negate()
            .and(this::atExtension));

    // ---Intake coral stack---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_CORAL_STACK,
        intakeCoralReq.and(() -> Robot.getCoralIntakeTarget() == CoralIntakeTarget.STACK));

    bindTransition(
        SuperState.INTAKE_CORAL_STACK,
        SuperState.READY_CORAL,
        new Trigger(arm::hasCoral).debounce(0.1));

    // ---L2---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L2,
        preScoreReq.and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2));

    bindTransition(SuperState.PRE_L2, SuperState.SCORE_L2, scoreReq.and(this::atExtension));

    bindTransition(
        SuperState.SCORE_L2,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    // ---L3---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L3,
        preScoreReq.and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3));

    bindTransition(SuperState.PRE_L3, SuperState.SCORE_L3, scoreReq.and(this::atExtension));

    bindTransition(
        SuperState.SCORE_L3,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    // ---L4---
    bindTransition(
        SuperState.READY_CORAL,
        SuperState.PRE_L4,
        preScoreReq.and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3));

    bindTransition(SuperState.PRE_L4, SuperState.SCORE_L4, scoreReq.and(this::atExtension));

    bindTransition(
        SuperState.SCORE_L4,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    // ---Intake Algae Ground---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_GROUND,
        intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.GROUND));

    bindTransition(
        SuperState.INTAKE_ALGAE_GROUND,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Intake Algae Stack---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_STACK,
        intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.STACK));

    bindTransition(
        SuperState.INTAKE_ALGAE_STACK,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Intake Algae Low---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_LOW,
        intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW));

    bindTransition(
        SuperState.INTAKE_ALGAE_LOW,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Intake Algae High---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_HIGH,
        intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH));

    bindTransition(
        SuperState.INTAKE_ALGAE_HIGH,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Score Barge---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_BARGE,
        preScoreReq.and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE));

    bindTransition(SuperState.PRE_BARGE, SuperState.SCORE_BARGE, scoreReq.and(this::atExtension));

    bindTransition(
        SuperState.SCORE_BARGE,
        SuperState.IDLE,
        // TODO i don't trust the state timer but i'm not sure if i can use the current check
        new Trigger(() -> stateTimer.hasElapsed(0.5)).and(arm::hasAlgae).negate().debounce(0.2));

    // ---Score Processor---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_PROCESSOR,
        preScoreReq.and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.PROCESSOR));

    bindTransition(
        SuperState.PRE_PROCESSOR, SuperState.SCORE_PROCESSOR, scoreReq.and(this::atExtension));

    bindTransition(
        SuperState.SCORE_PROCESSOR,
        SuperState.IDLE,
        new Trigger(arm::hasAlgae).negate().debounce(0.2).and(swerve::isNearProcessor).negate());

    // ---Climb---
    bindTransition(SuperState.IDLE, SuperState.PRE_CLIMB, preClimbReq);

    bindTransition(SuperState.PRE_CLIMB, SuperState.CLIMB, climbConfReq.and(climber::atExtension));

    bindTransition(SuperState.CLIMB, SuperState.PRE_CLIMB, climbCancelReq);

    bindTransition(SuperState.PRE_CLIMB, SuperState.IDLE, climbCancelReq);
  }

  public SuperState getState() {
    return state;
  }

  public boolean stateIsCoral() {
    return state.isCoral();
  }

  public boolean stateIsIntakeAlgaeReef() {
    return state == SuperState.INTAKE_ALGAE_HIGH || state == SuperState.INTAKE_ALGAE_LOW;
  }

  public boolean stateIsIdle() {
    return state == SuperState.IDLE;
  }

  public boolean stateIsProcessor() {
    return state == SuperState.READY_ALGAE
        || state == SuperState.PRE_PROCESSOR
        || state == SuperState.SCORE_PROCESSOR;
  }

  public boolean stateIsBarge() {
    return state == SuperState.READY_ALGAE
        || state == SuperState.PRE_BARGE
        || state == SuperState.SCORE_BARGE;
  }
}
