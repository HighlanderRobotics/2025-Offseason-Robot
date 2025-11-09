// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    // "right handoff" means the robot is about to score on its right, meaning the arm goes to the
    // left
    RIGHT_PRE_PRE_HANDOFF(
        ElevatorState.PRE_HANDOFF, ArmState.PRE_RIGHT_HANDOFF, IntakeState.READY_CORAL_INTAKE),
    RIGHT_PRE_HANDOFF(
        ElevatorState.HANDOFF, ArmState.RIGHT_HANDOFF, IntakeState.READY_CORAL_INTAKE),
    RIGHT_HANDOFF(ElevatorState.HANDOFF, ArmState.RIGHT_HANDOFF, IntakeState.HANDOFF),
    // this is to make it "take the long way around". it's kind of stupid but
    RIGHT_POST_HANDOFF(
        ElevatorState.RIGHT_POST_HANDOFF, ArmState.RIGHT_POST_HANDOFF, IntakeState.HANDOFF),

    // "left handoff" means the robot is about to score on its left, meaning the arm goes to the
    // right
    LEFT_PRE_PRE_HANDOFF(
        ElevatorState.PRE_HANDOFF, ArmState.PRE_LEFT_HANDOFF, IntakeState.READY_CORAL_INTAKE),
    LEFT_PRE_HANDOFF(ElevatorState.HANDOFF, ArmState.LEFT_HANDOFF, IntakeState.READY_CORAL_INTAKE),
    LEFT_HANDOFF(ElevatorState.HANDOFF, ArmState.LEFT_HANDOFF, IntakeState.HANDOFF),
    // this is to make it "take the long way around". it's kind of stupid but
    LEFT_POST_HANDOFF(
        ElevatorState.LEFT_POST_HANDOFF, ArmState.LEFT_POST_HANDOFF, IntakeState.HANDOFF),

    INTAKE_CORAL_STACK(
        ElevatorState.INTAKE_CORAL_STACK, ArmState.INTAKE_CORAL_STACK, IntakeState.CLIMB),
    READY_CORAL_ARM(ElevatorState.IDLE, ArmState.READY_CORAL_ARM, IntakeState.IDLE),

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

    // INTAKE_ALGAE_HIGH_RIGHT(
    //     ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF_RIGHT,
    // IntakeState.IDLE),
    // INTAKE_ALGAE_LOW_RIGHT(
    //     ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF_RIGHT, IntakeState.IDLE),

    // INTAKE_ALGAE_HIGH_LEFT(
    //     ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF_LEFT, IntakeState.IDLE),
    // INTAKE_ALGAE_LOW_LEFT(
    //     ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF_LEFT, IntakeState.IDLE),

    // INTAKE_ALGAE_STACK(
    //     ElevatorState.INTAKE_ALGAE_STACK, ArmState.INTAKE_ALGAE_STACK, IntakeState.IDLE),
    // INTAKE_ALGAE_GROUND(
    //     ElevatorState.INTAKE_ALGAE_GROUND, ArmState.INTAKE_ALGAE_GROUND, IntakeState.IDLE),

    // READY_ALGAE(ElevatorState.READY_ALGAE, ArmState.READY_ALGAE, IntakeState.IDLE),

    // PRE_BARGE_RIGHT(ElevatorState.BARGE, ArmState.PRE_BARGE_RIGHT, IntakeState.IDLE),
    // SCORE_BARGE_RIGHT(ElevatorState.BARGE, ArmState.SCORE_BARGE_RIGHT, IntakeState.IDLE),

    // PRE_BARGE_LEFT(ElevatorState.BARGE, ArmState.PRE_BARGE_LEFT, IntakeState.IDLE),
    // SCORE_BARGE_LEFT(ElevatorState.BARGE, ArmState.SCORE_BARGE_LEFT, IntakeState.IDLE),

    // // processor is left side only
    // PRE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.PRE_PROCESSOR, IntakeState.IDLE),
    // SCORE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.SCORE_PROCESSOR, IntakeState.IDLE),

    PRE_CLIMB(
        ElevatorState.PRE_CLIMB, ArmState.PRE_CLIMB, IntakeState.CLIMB, ClimberState.PRE_CLIMB),
    CLIMB(ElevatorState.CLIMB, ArmState.CLIMB, IntakeState.CLIMB, ClimberState.CLIMB);

    public final ElevatorState elevatorState;
    public final ArmState armState;
    public final IntakeState intakeState;
    public final ClimberState climberState;
    public final Trigger trigger;

    private SuperState(
        ElevatorState elevatorState,
        ArmState armState,
        IntakeState intakeState,
        ClimberState climberState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.climberState = climberState;
      trigger = new Trigger(() -> state == this);
    }

    private SuperState(ElevatorState elevatorState, ArmState armState, IntakeState intakeState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.climberState = ClimberState.IDLE;
      trigger = new Trigger(() -> state == this);
    }

    public Trigger getTrigger() {
      return trigger;
    }

    public boolean isCoral() {
      return this == INTAKE_CORAL_GROUND
          || this == READY_CORAL_INTAKE
          || this == RIGHT_HANDOFF
          || this == LEFT_HANDOFF
          || this == INTAKE_CORAL_STACK
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

    public boolean isScoreCoralRight() {
      return this == PRE_L2_RIGHT
          || this == SCORE_L2_RIGHT
          || this == PRE_L3_RIGHT
          || this == SCORE_L3_RIGHT
          || this == PRE_L4_RIGHT
          || this == SCORE_L4_RIGHT;
    }

    public boolean isScoreCoralLeft() {
      return this == PRE_L2_LEFT
          || this == SCORE_L2_LEFT
          || this == PRE_L3_LEFT
          || this == SCORE_L3_LEFT
          || this == PRE_L4_LEFT
          || this == SCORE_L4_LEFT;
    }

    // public boolean isAlgae() {
    //   return this == INTAKE_ALGAE_HIGH_RIGHT
    //       || this == INTAKE_ALGAE_LOW_RIGHT
    //       || this == INTAKE_ALGAE_STACK
    //       || this == INTAKE_ALGAE_GROUND
    //       || this == READY_ALGAE
    //       || this == PRE_BARGE_RIGHT
    //       || this == SCORE_BARGE_RIGHT
    //       || this == PRE_PROCESSOR
    //       || this == SCORE_PROCESSOR;
    // }
  }

  @AutoLogOutput(key = "Superstructure/State")
  private static SuperState state = SuperState.IDLE;

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

  //   @AutoLogOutput(key = "Superstructure/Algae Intake Request")
  //   public Trigger intakeAlgaeReq;

  @AutoLogOutput(key = "Superstructure/Pre Climb Request")
  public Trigger preClimbReq;

  @AutoLogOutput(key = "Superstructure/Climb Confirm Request")
  public Trigger climbConfReq;

  @AutoLogOutput(key = "Superstructure/Climb Cancel Request")
  public Trigger climbCancelReq;

  @AutoLogOutput(key = "Superstructure/At Extension?")
  public Trigger atExtensionTrigger = new Trigger(this::atExtension).or(Robot::isSimulation);

  @AutoLogOutput(key = "Superstructure/Intake Has Game Piece?")
  public Trigger intakeHasGamePieceTrigger;

  // May need to distinguish between coral and algae
  @AutoLogOutput(key = "Superstructure/Arm Has Game Piece?")
  public Trigger armHasGamePieceTrigger;

  // i'm fully aware this is an awful name
  @AutoLogOutput(key = "Superstructure/Away From Reef?")
  public Trigger awayFromReefTrigger;

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
    preScoreReq = driver.rightTrigger().or(Autos.autoPreScoreReq);

    scoreReq = driver.rightTrigger().negate().and(DriverStation::isTeleop).or(Autos.autoScoreReq);

    intakeCoralReq = driver.leftTrigger().or(Autos.autoIntakeCoralReq);

    // intakeAlgaeReq = driver.leftBumper().or(Autos.autoIntakeAlgaeReq);

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

    intakeHasGamePieceTrigger = new Trigger(intake::hasGamePiece);

    armHasGamePieceTrigger = new Trigger(arm::hasGamePiece);

    awayFromReefTrigger = new Trigger(swerve::isNearL1Reef).negate();
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
    trigger.and(start.getTrigger()).onTrue(changeStateTo(end));
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
    trigger.and(start.getTrigger()).onTrue(Commands.parallel(changeStateTo(end), cmd));
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
              this.prevState = state;
              state = nextState;
              setSubstates();
            })
        .ignoringDisable(true)
        .withName("State Change Command");
  }

  private void setSubstates() {
    elevator.setState(state.elevatorState);
    arm.setState(state.armState);
    intake.setState(state.intakeState);
    climber.setState(state.climberState);
  }

  public Command transitionAfterZeroing() {
    return Commands.runOnce(
            () -> {
              SuperState target;
              // cant distinguish between coral and algae rn
              if (intake.hasGamePiece()) {
                target = SuperState.READY_CORAL_INTAKE;
                //   } else if (arm.hasAlgae) {
                //     target = SuperState.READY_ALGAE;
              } else if (arm.hasCoral) {
                target = SuperState.READY_CORAL_ARM;
              } else {
                target = SuperState.IDLE;
              }
              System.out.println("Transitioning to " + target + " after zeroing");
              changeStateTo(target).schedule();
              ;
            })
        .ignoringDisable(true);
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
        intakeHasGamePieceTrigger.debounce(0.1));

    // ---Cancel intake coral ground---
    bindTransition(
        SuperState.INTAKE_CORAL_GROUND,
        SuperState.IDLE,
        intakeCoralReq.negate().and(intakeHasGamePieceTrigger.negate()));

    // ---In case coral drops from the intake for some reason---
    bindTransition(
        SuperState.READY_CORAL_INTAKE,
        SuperState.IDLE,
        intakeHasGamePieceTrigger.negate().debounce(0.5));

    // ---L1---
    bindTransition(
        SuperState.READY_CORAL_INTAKE,
        SuperState.PRE_L1,
        atExtensionTrigger
            .debounce(0.1)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L1)
            .and(preScoreReq));

    bindTransition(SuperState.PRE_L1, SuperState.L1, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.L1,
        SuperState.IDLE,
        intakeHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Right Handoff---
    bindTransition(
        SuperState.READY_CORAL_INTAKE,
        SuperState.RIGHT_PRE_PRE_HANDOFF,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() != CoralScoreTarget.L1)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.RIGHT_PRE_PRE_HANDOFF,
        SuperState.RIGHT_PRE_HANDOFF,
        atExtensionTrigger.debounce(0.1).and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.RIGHT_PRE_HANDOFF,
        SuperState.RIGHT_HANDOFF,
        atExtensionTrigger.debounce(0.25).and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.RIGHT_HANDOFF,
        SuperState.RIGHT_POST_HANDOFF,
        armHasGamePieceTrigger
            .debounce(0.1)
            .and(intakeHasGamePieceTrigger.negate().debounce(0.05))
            .and(atExtensionTrigger));

    // ---Left Handoff---
    bindTransition(
        SuperState.READY_CORAL_INTAKE,
        SuperState.LEFT_PRE_PRE_HANDOFF,
        preScoreReq
            .and(() -> Robot.getCoralScoreTarget() != CoralScoreTarget.L1)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.LEFT_PRE_PRE_HANDOFF,
        SuperState.LEFT_PRE_HANDOFF,
        atExtensionTrigger.debounce(0.1).and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.LEFT_PRE_HANDOFF,
        SuperState.LEFT_HANDOFF,
        atExtensionTrigger.debounce(0.25).and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.LEFT_HANDOFF,
        SuperState.LEFT_POST_HANDOFF,
        armHasGamePieceTrigger
            .debounce(0.1)
            .and(intakeHasGamePieceTrigger.negate().debounce(0.05))
            .and(atExtensionTrigger));

    // ---Intake coral stack---
    // No intake coral stack -> L1 cause why would you do that
    // READY_CORAL_ARM is the counterpart to READY_CORAL_INTAKE, as in it's essentially idle except
    // it has a coral
    // TODO maybe add intake coral straight to l4 or smth for auto (only?)
    bindTransition(
        SuperState.IDLE,
        SuperState.INTAKE_CORAL_STACK,
        intakeCoralReq.and(() -> Robot.getCoralIntakeTarget() == CoralIntakeTarget.STACK));

    bindTransition(SuperState.INTAKE_CORAL_STACK, SuperState.IDLE, intakeCoralReq.negate());

    bindTransition(
        SuperState.INTAKE_CORAL_STACK,
        SuperState.READY_CORAL_ARM,
        armHasGamePieceTrigger.debounce(0.1));

    // ---In case coral drops from the arm for some reason
    bindTransition(
        SuperState.READY_CORAL_ARM, SuperState.IDLE, armHasGamePieceTrigger.negate().debounce(0.5));

    // ---Right L2---
    bindTransition(
        SuperState.RIGHT_POST_HANDOFF,
        SuperState.PRE_L2_RIGHT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L2_RIGHT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L2_RIGHT, SuperState.SCORE_L2_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L2_RIGHT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Left L2---
    bindTransition(
        SuperState.LEFT_POST_HANDOFF,
        SuperState.PRE_L2_LEFT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L2_LEFT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L2)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L2_LEFT, SuperState.SCORE_L2_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L2_LEFT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Right L3---
    bindTransition(
        SuperState.RIGHT_POST_HANDOFF,
        SuperState.PRE_L3_RIGHT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L3_RIGHT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L3_RIGHT, SuperState.SCORE_L3_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L3_RIGHT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Left L3---
    bindTransition(
        SuperState.LEFT_POST_HANDOFF,
        SuperState.PRE_L3_LEFT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L3_LEFT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L3)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L3_LEFT, SuperState.SCORE_L3_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L3_LEFT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Right L4---
    bindTransition(
        SuperState.RIGHT_POST_HANDOFF,
        SuperState.PRE_L4_RIGHT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L4_RIGHT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    bindTransition(
        SuperState.PRE_L4_RIGHT, SuperState.SCORE_L4_RIGHT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L4_RIGHT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Left L4---
    bindTransition(
        SuperState.LEFT_POST_HANDOFF,
        SuperState.PRE_L4_LEFT,
        atExtensionTrigger
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.READY_CORAL_ARM,
        SuperState.PRE_L4_LEFT,
        preScoreReq
            .and(atExtensionTrigger)
            .and(() -> Robot.getCoralScoreTarget() == CoralScoreTarget.L4)
            .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    bindTransition(
        SuperState.PRE_L4_LEFT, SuperState.SCORE_L4_LEFT, scoreReq.and(atExtensionTrigger));

    bindTransition(
        SuperState.SCORE_L4_LEFT,
        SuperState.IDLE,
        armHasGamePieceTrigger.negate().debounce(0.1).and(awayFromReefTrigger.debounce(0.15)));

    // ---Intake Algae Ground---
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_GROUND,
    //     intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.GROUND));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_GROUND,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel intake algae ground
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_GROUND,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---Intake Algae Stack---
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_STACK,
    //     intakeAlgaeReq.and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.STACK));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_STACK,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel intake algae stack
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_STACK,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---Right Intake Algae Low---
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_LOW_RIGHT,
    //     intakeAlgaeReq
    //         .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_LOW_RIGHT,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel right intake algae low---
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_LOW_RIGHT,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---Left Intake Algae Low---
    // // might hit climber?
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_LOW_LEFT,
    //     intakeAlgaeReq
    //         .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_LOW_LEFT,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel left intake algae low---
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_LOW_LEFT,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---Right Intake Algae High---
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_HIGH_RIGHT,
    //     intakeAlgaeReq
    //         .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_HIGH_RIGHT,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel right intake algae high---
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_HIGH_RIGHT,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---Left Intake Algae High---
    // bindTransition(
    //     SuperState.IDLE,
    //     SuperState.INTAKE_ALGAE_HIGH_LEFT,
    //     intakeAlgaeReq
    //         .and(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    // bindTransition(
    //     SuperState.INTAKE_ALGAE_HIGH_LEFT,
    //     SuperState.READY_ALGAE,
    //     armHasGamePieceTrigger.debounce(0.1));

    // // ---Cancel left intake algae high---
    // bindTransition(
    //     SuperState.INTAKE_ALGAE_HIGH_LEFT,
    //     SuperState.IDLE,
    //     intakeAlgaeReq.negate().and(armHasGamePieceTrigger.negate()));

    // // ---In case algae drops from the arm for some reason
    // bindTransition(
    //     SuperState.READY_ALGAE, SuperState.IDLE, armHasGamePieceTrigger.negate().debounce(0.5));

    // // ---Right Score Barge---
    // bindTransition(
    //     SuperState.READY_ALGAE,
    //     SuperState.PRE_BARGE_RIGHT,
    //     preScoreReq
    //         .and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.RIGHT));

    // bindTransition(
    //     SuperState.PRE_BARGE_RIGHT, SuperState.SCORE_BARGE_RIGHT,
    // scoreReq.and(atExtensionTrigger));

    // bindTransition(
    //     SuperState.SCORE_BARGE_RIGHT,
    //     SuperState.IDLE,
    //     // TODO i don't trust the state timer but i'm not sure if i can use the current check
    //     new Trigger(() -> stateTimer.hasElapsed(0.5))
    //         .and(armHasGamePieceTrigger.negate())
    //         .debounce(0.2));

    // // ---Left Score Barge---
    // bindTransition(
    //     SuperState.READY_ALGAE,
    //     SuperState.PRE_BARGE_LEFT,
    //     preScoreReq
    //         .and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
    //         .and(() -> Robot.getScoringSide() == ScoringSide.LEFT));

    // bindTransition(
    //     SuperState.PRE_BARGE_LEFT, SuperState.SCORE_BARGE_LEFT,
    // scoreReq.and(atExtensionTrigger));

    // bindTransition(
    //     SuperState.SCORE_BARGE_LEFT,
    //     SuperState.IDLE,
    //     // TODO i don't trust the state timer but i'm not sure if i can use the current check
    //     new Trigger(() -> stateTimer.hasElapsed(0.5))
    //         .and(armHasGamePieceTrigger.negate())
    //         .debounce(0.2));

    // // ---Score Processor---
    // bindTransition(
    //     SuperState.READY_ALGAE,
    //     SuperState.PRE_PROCESSOR,
    //     preScoreReq.and(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.PROCESSOR));

    // bindTransition(
    //     SuperState.PRE_PROCESSOR, SuperState.SCORE_PROCESSOR, scoreReq.and(atExtensionTrigger));

    // bindTransition(
    //     SuperState.SCORE_PROCESSOR,
    //     SuperState.IDLE,
    //     armHasGamePieceTrigger
    //         .negate()
    //         .debounce(0.2)
    //         .and(new Trigger(swerve::nearProcessor).negate()));

    // ---Climb---
    bindTransition(SuperState.IDLE, SuperState.PRE_CLIMB, preClimbReq);

    bindTransition(
        SuperState.PRE_CLIMB, SuperState.CLIMB, climbConfReq.and(climber::atClimbExtension));

    // ---Cancel climb---
    bindTransition(SuperState.CLIMB, SuperState.PRE_CLIMB, climbCancelReq);

    bindTransition(SuperState.PRE_CLIMB, SuperState.IDLE, climbCancelReq);
  }

  /**
   * <b>Only for setting initial state at the beginning of auto</b>
   *
   * @param state the state to set to
   */
  public void resetStateForAuto(SuperState nextState) {
    System.out.println("Resetting state from " + state + " to " + nextState + " for auto.");
    stateTimer.reset();
    this.prevState = state;
    state = nextState;
    setSubstates();
  }

  public static SuperState getState() {
    return state;
  }

  public boolean stateIsCoral() {
    return getState().isCoral();
  }

  //   public boolean stateIsAlgae() {
  //     return getState().isAlgae();
  //   }

  //   public boolean stateIsIntakeAlgaeReef() {
  //     return getState() == SuperState.INTAKE_ALGAE_HIGH_RIGHT
  //         || getState() == SuperState.INTAKE_ALGAE_LOW_RIGHT;
  //   }

  //   public static boolean stateIsVoltageControl() {
  //     return state == SuperState.INTAKE_ALGAE_HIGH_RIGHT
  //         || state == SuperState.INTAKE_ALGAE_LOW_RIGHT
  //         || state == SuperState.INTAKE_ALGAE_HIGH_LEFT
  //         || state == SuperState.INTAKE_ALGAE_LOW_LEFT
  //         || state == SuperState.INTAKE_ALGAE_GROUND
  //         || state == SuperState.INTAKE_ALGAE_STACK
  //         || state == SuperState.READY_ALGAE;
  //   }

  public boolean stateIsIdle() {
    return getState() == SuperState.IDLE;
  }

  //   public boolean stateIsProcessor() {
  //     return getState() == SuperState.READY_ALGAE
  //         || getState() == SuperState.PRE_PROCESSOR
  //         || getState() == SuperState.SCORE_PROCESSOR;
  //   }

  //   public boolean stateIsBarge() {
  //     return getState() == SuperState.READY_ALGAE
  //         || getState() == SuperState.PRE_BARGE_RIGHT
  //         || getState() == SuperState.SCORE_BARGE_RIGHT;
  //   }
}
