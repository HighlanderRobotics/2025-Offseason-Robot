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
import frc.robot.Robot.ScoringSide;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. See <a
   * href=https://docs.google.com/document/d/1l7MigoxtVseHWCiXcUjIAOOn5Q1dl7hid5luzBnOgzE/edit?tab=t.0>
   * this document</a> for screenshots of the robot in each state. There are also named positions in
   * cad for each state.
   */
  public enum SuperState {
    IDLE(ElevatorState.IDLE, ArmState.IDLE, IntakeState.IDLE),

    INTAKE_CORAL_GROUND(ElevatorState.IDLE, ArmState.IDLE, IntakeState.INTAKE_CORAL),

    READY_CORAL_INTAKE(ElevatorState.IDLE, ArmState.IDLE, IntakeState.READY_CORAL_INTAKE),
    PRE_HANDOFF(ElevatorState.HANDOFF, ArmState.HANDOFF, IntakeState.READY_CORAL_INTAKE),
    HANDOFF(ElevatorState.HANDOFF, ArmState.HANDOFF, IntakeState.HANDOFF),
    READY_CORAL_ARM(ElevatorState.IDLE, ArmState.READY_CORAL_ARM, IntakeState.IDLE),

    INTAKE_CORAL_STACK(
        ElevatorState.INTAKE_CORAL_STACK, ArmState.INTAKE_CORAL_STACK, IntakeState.CLIMB),

    PRE_L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.PRE_L1),
    L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.SCORE_L1),

    PRE_L2_RIGHT(ElevatorState.PRE_L2, ArmState.PRE_L2_RIGHT, IntakeState.IDLE),
    SCORE_L2_RIGHT(ElevatorState.L2, ArmState.SCORE_L2_RIGHT, IntakeState.IDLE),
    PRE_L3_RIGHT(ElevatorState.PRE_L3, ArmState.PRE_L3_RIGHT, IntakeState.IDLE),
    SCORE_L3_RIGHT(ElevatorState.L3, ArmState.SCORE_L3_RIGHT, IntakeState.IDLE),
    PRE_L4_RIGHT(ElevatorState.PRE_L4, ArmState.PRE_L4_RIGHT, IntakeState.IDLE),
    SCORE_L4_RIGHT(ElevatorState.L4, ArmState.SCORE_L4_RIGHT, IntakeState.IDLE),

    PRE_L2_LEFT(ElevatorState.PRE_L2, ArmState.PRE_L2_LEFT, IntakeState.IDLE),
    SCORE_L2_LEFT(ElevatorState.L2, ArmState.SCORE_L2_LEFT, IntakeState.IDLE),
    PRE_L3_LEFT(ElevatorState.PRE_L3, ArmState.PRE_L3_LEFT, IntakeState.IDLE),
    SCORE_L3_LEFT(ElevatorState.L3, ArmState.SCORE_L3_LEFT, IntakeState.IDLE),
    PRE_L4_LEFT(ElevatorState.PRE_L4, ArmState.PRE_L4_LEFT, IntakeState.IDLE),
    SCORE_L4_LEFT(ElevatorState.L4, ArmState.SCORE_L4_LEFT, IntakeState.IDLE),

    INTAKE_ALGAE_HIGH_RIGHT(
        ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF_RIGHT, IntakeState.IDLE),
    INTAKE_ALGAE_LOW_RIGHT(
        ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF_RIGHT, IntakeState.IDLE),

    INTAKE_ALGAE_HIGH_LEFT(
        ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF_LEFT, IntakeState.IDLE),
    INTAKE_ALGAE_LOW_LEFT(
        ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF_LEFT, IntakeState.IDLE),

    INTAKE_ALGAE_STACK(
        ElevatorState.INTAKE_ALGAE_STACK, ArmState.INTAKE_ALGAE_STACK, IntakeState.IDLE),
    INTAKE_ALGAE_GROUND(
        ElevatorState.INTAKE_ALGAE_GROUND, ArmState.INTAKE_ALGAE_GROUND, IntakeState.IDLE),

    READY_ALGAE(ElevatorState.READY_ALGAE, ArmState.READY_ALGAE, IntakeState.IDLE),

    PRE_BARGE_RIGHT(ElevatorState.BARGE, ArmState.PRE_BARGE_RIGHT, IntakeState.IDLE),
    SCORE_BARGE_RIGHT(ElevatorState.BARGE, ArmState.SCORE_BARGE_RIGHT, IntakeState.IDLE),

    PRE_BARGE_LEFT(ElevatorState.BARGE, ArmState.PRE_BARGE_LEFT, IntakeState.IDLE),
    SCORE_BARGE_LEFT(ElevatorState.BARGE, ArmState.SCORE_BARGE_LEFT, IntakeState.IDLE),

    // processor is left side only
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
          || this == READY_CORAL_INTAKE
          || this == HANDOFF
          || this == INTAKE_CORAL_STACK
          || this == READY_CORAL_ARM
          || this == PRE_L1
          || this == L1
          || this == PRE_L2_RIGHT
          || this == SCORE_L2_RIGHT
          || this == PRE_L3_RIGHT
          || this == SCORE_L3_RIGHT
          || this == PRE_L4_RIGHT
          || this == SCORE_L4_RIGHT
          || this == PRE_L2_LEFT
          || this == SCORE_L2_LEFT
          || this == PRE_L3_LEFT
          || this == SCORE_L3_LEFT
          || this == PRE_L4_LEFT
          || this == SCORE_L4_LEFT;
    }

    public boolean isScoreCoral() {
      return this == PRE_L1
          || this == L1
          || this == PRE_L2_RIGHT
          || this == SCORE_L2_RIGHT
          || this == PRE_L3_RIGHT
          || this == SCORE_L3_RIGHT
          || this == PRE_L4_RIGHT
          || this == SCORE_L4_RIGHT
          || this == PRE_L2_LEFT
          || this == SCORE_L2_LEFT
          || this == PRE_L3_LEFT
          || this == SCORE_L3_LEFT
          || this == PRE_L4_LEFT
          || this == SCORE_L4_LEFT;
    }

    public boolean isAlgae() {
      return this == INTAKE_ALGAE_HIGH_RIGHT
          || this == INTAKE_ALGAE_LOW_RIGHT
          || this == INTAKE_ALGAE_STACK
          || this == INTAKE_ALGAE_GROUND
          || this == READY_ALGAE
          || this == PRE_BARGE_RIGHT
          || this == SCORE_BARGE_RIGHT
          || this == PRE_PROCESSOR
          || this == SCORE_PROCESSOR;
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
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

  public Trigger atExtensionTrigger = new Trigger(this::atExtension);

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
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state
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
    // when 1) the robot is in the start state and 2) the trigger is true, the robot changes state
    // to the end state IN PARALLEL to running the command that got passed in
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
        SuperState.READY_CORAL_INTAKE,
        new Trigger(intake::hasCoral).debounce(0.1));

    // Handoff
    bindTransition(
        SuperState.READY_CORAL_INTAKE,
        SuperState.PRE_HANDOFF,
        // TODO maybe make the hascorals and stuff triggers inside intake?
        preScoreReq);

    bindTransition(
        SuperState.PRE_HANDOFF,
        SuperState.HANDOFF,
        // maybe this also needs prescore idk
        atExtensionTrigger);

    bindTransition(
        SuperState.HANDOFF,
        // uhhh may need another intermediate state
        SuperState.READY_CORAL_ARM,
        new Trigger(arm::hasCoral)
            .debounce(0.1)
            .and(intake::hasCoral)
            .negate()
            .and(atExtensionTrigger));

    // ---Intake coral stack---
    // No intake coral stack -> L1 cause why would you do that
    // TODO maybe add intake coral straight to l4 or smth for auto (only?)
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_CORAL_STACK,
        intakeCoralReq.and(() -> Robot.getCoralIntakeTarget() == CoralIntakeTarget.STACK));

    bindTransition(
        SuperState.INTAKE_CORAL_STACK,
        SuperState.READY_CORAL_ARM,
        new Trigger(arm::hasCoral).debounce(0.1));

    // ---L2---
    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L2_RIGHT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L2_RIGHT, SuperState.SCORE_L2_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L2_RIGHT,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L2_LEFT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L2_LEFT, SuperState.SCORE_L2_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L2_LEFT,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    // ---L3---
    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L3_RIGHT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L3_RIGHT, SuperState.SCORE_L3_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L3_RIGHT,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L3_LEFT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L3_LEFT, SuperState.SCORE_L3_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L3_LEFT,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    // ---L4---
    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L4_RIGHT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L4_RIGHT, SuperState.SCORE_L4_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L4_RIGHT,
        SuperState.IDLE,
        new Trigger(arm::hasCoral)
            .negate()
            // TODO this is a different near reef (?)
            .and(new Trigger(swerve::isNearL1Reef).negate().debounce(0.15)));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L4_LEFT,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L4_LEFT, SuperState.SCORE_L4_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L4_LEFT,
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
        SuperState.INTAKE_ALGAE_LOW_RIGHT,
        intakeAlgaeReq
            .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.INTAKE_ALGAE_LOW_RIGHT,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_LOW_LEFT,
        intakeAlgaeReq
            .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.INTAKE_ALGAE_LOW_LEFT,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Intake Algae High---
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_HIGH_RIGHT,
        intakeAlgaeReq
            .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.INTAKE_ALGAE_HIGH_RIGHT,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_ALGAE_HIGH_LEFT,
        intakeAlgaeReq
            .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.INTAKE_ALGAE_HIGH_LEFT,
        SuperState.READY_ALGAE,
        new Trigger(arm::hasAlgae).debounce(0.1));

    // ---Score Barge---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_BARGE_RIGHT,
        preScoreReq
            .and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_BARGE_RIGHT, SuperState.SCORE_BARGE_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_BARGE_RIGHT,
        SuperState.IDLE,
        // TODO i don't trust the state timer but i'm not sure if i can use the current check
        new Trigger(() -> stateTimer.hasElapsed(0.5)).and(arm::hasAlgae).negate().debounce(0.2));

    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_BARGE_LEFT,
        preScoreReq
            .and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_BARGE_LEFT, SuperState.SCORE_BARGE_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_BARGE_LEFT,
        SuperState.IDLE,
        // TODO i don't trust the state timer but i'm not sure if i can use the current check
        new Trigger(() -> stateTimer.hasElapsed(0.5)).and(arm::hasAlgae).negate().debounce(0.2));

    // ---Score Processor---
    bindTransition(
        SuperState.READY_ALGAE,
        SuperState.PRE_PROCESSOR,
        preScoreReq.and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.PROCESSOR));

    bindTransition(
        SuperState.PRE_PROCESSOR, SuperState.SCORE_PROCESSOR, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_PROCESSOR,
        SuperState.IDLE,
        new Trigger(arm::hasAlgae).negate().debounce(0.2).and(swerve::nearProcessor).negate());

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
    return state == SuperState.INTAKE_ALGAE_HIGH_RIGHT
        || state == SuperState.INTAKE_ALGAE_LOW_RIGHT;
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
        || state == SuperState.PRE_BARGE_RIGHT
        || state == SuperState.SCORE_BARGE_RIGHT;
  }
}
