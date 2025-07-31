// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.ArmSubsystem.ArmState;
import frc.robot.climb.ClimberSubsystem;
import frc.robot.climb.ClimberSubsystem.ClimberState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.IntakeSubsystem.IntakeState;
import frc.robot.routing.RoutingSubsystem;
import frc.robot.routing.RoutingSubsystem.RoutingState;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. Hopefully we can get named
   * positions set up in cad to make this easier?
   */
  public enum SuperState {
    IDLE(ElevatorState.IDLE, ArmState.IDLE, IntakeState.IDLE, RoutingState.IDLE),
    PRE_INTAKE_CORAL(
        ElevatorState.PRE_INTAKE_CORAL_GROUND,
        ArmState.INTAKE_CORAL_GROUND,
        IntakeState.INTAKE,
        RoutingState.INTAKE),
    INTAKE_CORAL(
        ElevatorState.INTAKE_CORAL_GROUND,
        ArmState.INTAKE_CORAL_GROUND,
        IntakeState.INTAKE,
        RoutingState.INTAKE),
    POST_INTAKE_CORAL(
        ElevatorState.POST_INTAKE_CORAL_GROUND,
        ArmState.IDLE,
        IntakeState.INTAKE,
        RoutingState.INTAKE), // TODO double check this
    READY_CORAL(ElevatorState.IDLE, ArmState.IDLE, IntakeState.IDLE, RoutingState.IDLE),
    PRE_L1(ElevatorState.L1, ArmState.PRE_L1),
    L1(ElevatorState.L1, ArmState.L1),
    POST_L1(ElevatorState.L1, ArmState.IDLE),
    PRE_L2(ElevatorState.PRE_L2, ArmState.PRE_L2),
    L2(ElevatorState.L2, ArmState.L2),
    POST_L2(ElevatorState.PRE_L2, ArmState.PRE_L2),
    PRE_L3(ElevatorState.PRE_L3, ArmState.PRE_L3),
    L3(ElevatorState.L3, ArmState.L3),
    POST_L3(ElevatorState.PRE_L3, ArmState.PRE_L3),
    PRE_L4(ElevatorState.PRE_L4, ArmState.PRE_L4),
    L4(ElevatorState.L4, ArmState.L4),
    POST_L4(ElevatorState.POST_L4, ArmState.PRE_L4),
    INTAKE_ALGAE_REEF_HIGH(ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF),
    INTAKE_ALGAE_REEF_LOW(ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF),
    INTAKE_ALGAE_GROUND(ElevatorState.INTAKE_ALGAE_GROUND, ArmState.INTAKE_ALGAE_GROUND),
    POST_INTAKE_ALGAE_GROUND(ElevatorState.POST_INTAKE_ALGAE_GROUND, ArmState.IDLE),
    READY_ALGAE(ElevatorState.IDLE, ArmState.IDLE),
    PRE_BARGE(ElevatorState.BARGE, ArmState.PRE_BARGE),
    BARGE(ElevatorState.BARGE, ArmState.BARGE),
    PRE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.PRE_PROCESSOR),
    PROCESSOR(ElevatorState.PROCESSOR, ArmState.PROCESSOR),
    PRE_CLIMB(
        ElevatorState.IDLE,
        ArmState.CLIMB,
        IntakeState.INTAKE,
        RoutingState.IDLE,
        ClimberState.PRE_CLIMB),
    CLIMB(
        ElevatorState.IDLE,
        ArmState.CLIMB,
        IntakeState.INTAKE,
        RoutingState.IDLE,
        ClimberState.CLIMB);

    public final ElevatorState elevatorState;
    public final ArmState armState;
    public final IntakeState intakeState;
    public final RoutingState routingState;
    public final ClimberState climberState;

    private SuperState(
        ElevatorState elevatorState,
        ArmState armState,
        IntakeState intakeState,
        RoutingState routingState,
        ClimberState climberState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.routingState = routingState;
      this.climberState = climberState;
    }

    private SuperState(
        ElevatorState elevatorState,
        ArmState armState,
        IntakeState intakeState,
        RoutingState routingState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.routingState = routingState;
      this.climberState = ClimberState.IDLE;
    }

    private SuperState(ElevatorState elevatorState, ArmState armState, IntakeState intakeState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = intakeState;
      this.routingState = RoutingState.IDLE;
      this.climberState = ClimberState.IDLE;
    }

    private SuperState(ElevatorState elevatorState, ArmState armState) {
      this.elevatorState = elevatorState;
      this.armState = armState;
      this.intakeState = IntakeState.IDLE;
      this.routingState = RoutingState.IDLE;
      this.climberState = ClimberState.IDLE;
    }
  }

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;
  private final RoutingSubsystem routing;
  private final ClimberSubsystem climber;

  /** Creates a new Superstructure. */
  public Superstructure(
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      IntakeSubsystem intake,
      RoutingSubsystem routing,
      ClimberSubsystem climber) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.routing = routing;
    this.climber = climber;

    addTransitions();
  }

  public void periodic() {
    Logger.recordOutput("Superstructure/Superstructure State", state);
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

  private void addTransitions() {
    // ----Intake coral----
    bindTransition(
        SuperState.IDLE,
        SuperState.PRE_INTAKE_CORAL,
        Robot.intakeCoralReq.and(() -> !arm.hasPiece()));

    bindTransition(
            SuperState.PRE_INTAKE_CORAL,
            SuperState.INTAKE_CORAL,
            new Trigger(this::atExtension).and(intake::hasCoral));

    bindTransition(
            SuperState.INTAKE_CORAL,
            SuperState.POST_INTAKE_CORAL,
            new Trigger(arm::hasPiece).and(() -> !Robot.intakeCoralReq.getAsBoolean()));

    bindTransition(
            SuperState.POST_INTAKE_CORAL,
            SuperState.READY_CORAL,
            new Trigger(this::atExtension));

    // ----L1----
    bindTransition(
            SuperState.READY_CORAL,
            SuperState.PRE_L1,
            new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L1)
                .and(arm::hasPiece)
                .and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_L1,
            SuperState.L1,
            new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L1)
                .and(arm::hasPiece)
                .and(Robot.scoreReq));

    bindTransition(
            SuperState.L1,
            SuperState.POST_L1,
            new Trigger(
                () ->
                    !arm.hasPiece()
                        && !Robot.scoreReq
                            .getAsBoolean())); // also might need to check that it's backed away
    // from the reef?

    bindTransition(
            SuperState.POST_L1,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasPiece()).and(this::atExtension).and(() -> !Robot.scoreReq.getAsBoolean()));

    // ----L2----
    bindTransition(
            SuperState.READY_CORAL,
            SuperState.PRE_L2,
            new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L2).and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_L2,
            SuperState.L2,
            new Trigger(this::atExtension).and(Robot.scoreReq));

    bindTransition(
            SuperState.L2,
            SuperState.POST_L2,
            new Trigger(
                () ->
                    !Robot.scoreReq
                        .getAsBoolean())); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    bindTransition(
            SuperState.POST_L2,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasPiece()).and(this::atExtension).and(() -> !Robot.scoreReq.getAsBoolean()));

    // ----L3----
    bindTransition(
            SuperState.READY_CORAL,
            SuperState.PRE_L3,
            new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L3).and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_L3,
            SuperState.L3,
            new Trigger(this::atExtension).and(Robot.scoreReq));

    bindTransition(
            SuperState.L3,
            SuperState.POST_L3,
            new Trigger(
                () ->
                    !Robot.scoreReq
                        .getAsBoolean())); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    bindTransition(
            SuperState.POST_L3,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasPiece()).and(this::atExtension).and(() -> !Robot.scoreReq.getAsBoolean()));

    // ----L4----
    bindTransition(
            SuperState.READY_CORAL,
            SuperState.PRE_L4,
            new Trigger(() -> Robot.getCoralTarget() == ReefTarget.L4).and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_L4,
            SuperState.L4,
            new Trigger(this::atExtension).and(Robot.scoreReq));

    bindTransition(
            SuperState.L4,
            SuperState.POST_L4,
            new Trigger(() -> !Robot.scoreReq.getAsBoolean())
                .and(
                    () -> !arm.hasPiece())); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    bindTransition(
            SuperState.POST_L4,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasPiece()).and(this::atExtension).and(() -> !Robot.scoreReq.getAsBoolean()));

    // ---Intake Algae---
    bindTransition(
            SuperState.IDLE,
            SuperState.INTAKE_ALGAE_REEF_HIGH,
            Robot.intakeAlgaeReq
                .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.HIGH))
                .and(() -> !arm.hasPiece())); // TODO should probably do better checks for specific
    // pieces (prob what the last intakereq was)

    bindTransition(
            SuperState.INTAKE_ALGAE_REEF_HIGH,
            SuperState.READY_ALGAE,
            new Trigger(
                this::atExtension).and(arm::hasPiece).and(() -> !Robot.intakeAlgaeReq.getAsBoolean()));

    bindTransition(
            SuperState.IDLE,
            SuperState.INTAKE_ALGAE_REEF_LOW,
            Robot.intakeAlgaeReq
                .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.LOW))
                .and(() -> !arm.hasPiece())); // TODO should probably do better checks for specific
    // pieces (prob what the last intakereq was)

    bindTransition(
            SuperState.INTAKE_ALGAE_REEF_LOW,
            SuperState.READY_ALGAE,
            new Trigger(
                this::atExtension).and(arm::hasPiece).and(() -> !Robot.intakeAlgaeReq.getAsBoolean()));

    bindTransition(
            SuperState.IDLE,
            SuperState.INTAKE_ALGAE_GROUND,
            Robot.intakeAlgaeReq
                .and(new Trigger(() -> Robot.getAlgaeIntakeTarget() == AlgaeIntakeTarget.GROUND))
                .and(() -> !arm.hasPiece())); // TODO should probably do better checks for specific
    // pieces (prob what the last intakereq was)

    bindTransition(
            SuperState.INTAKE_ALGAE_GROUND,
            SuperState.POST_INTAKE_ALGAE_GROUND,
            new Trigger(arm::hasPiece).and(() -> !Robot.intakeAlgaeReq.getAsBoolean()));

    bindTransition(
            SuperState.POST_INTAKE_ALGAE_GROUND,
            SuperState.READY_ALGAE,
            new Trigger(this::atExtension));

    // ---Score algae--
    bindTransition(
            SuperState.READY_ALGAE,
            SuperState.PRE_PROCESSOR,
            new Trigger(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.PROCESSOR)
                .and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_PROCESSOR,
            SuperState.PROCESSOR,
            new Trigger(this::atExtension).and(Robot.scoreReq));

    bindTransition(
            SuperState.PROCESSOR,
            SuperState.IDLE,
            new Trigger(() -> !Robot.scoreReq.getAsBoolean()).and(() -> !arm.hasPiece()));

    // TODO this assumes, perhaps naively, that there's no intermediate barge extension!
    bindTransition(
            SuperState.READY_ALGAE,
            SuperState.PRE_BARGE,
            new Trigger(() -> Robot.getAlgaeScoreTarget() == AlgaeScoreTarget.BARGE)
                .and(Robot.preScoreReq));

    bindTransition(
            SuperState.PRE_BARGE,
            SuperState.BARGE,
            new Trigger(this::atExtension).and(Robot.scoreReq));

    bindTransition(
            SuperState.BARGE,
            SuperState.IDLE,
            new Trigger(() -> !Robot.scoreReq.getAsBoolean()).and(() -> !arm.hasPiece()));

    // ---Climb---
    // transitions.add(new Transition(SuperState.IDLE, SuperState.PRE_CLIMB, Robot.preClimbReq));

    // transitions.add(
    //     new Transition(
    //         SuperState.PRE_CLIMB,
    //         SuperState.CLIMB,
    //         Robot.climbReq.and(() -> this.atExtension())); // check climber current?

  }

  private Command changeStateTo(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
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
    routing.setState(state.routingState);
    climber.setState(state.climberState);
  }

  // add check for climber too maybe
  private boolean atExtension(SuperState state) {
    return elevator.atExtension(state.elevatorState.getExtensionMeters())
        && arm.atAngle(state.armState.getPivotAngle())
        && intake.atAngle(state.intakeState.getPivotAngle());
  }

  private boolean atExtension() {
    return atExtension(state);
  }

  public SuperState getState() {
    return state;
  }
}
