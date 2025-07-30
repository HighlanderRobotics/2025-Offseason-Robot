// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  /**
   * We should have a state for every single "pose" the robot will hit. Hopefully we can get named
   * positions set up in cad to make this easier?
   */
  public enum SuperState {
    IDLE(ElevatorState.IDLE, ArmState.IDLE, IntakeState.IDLE, RoutingState.IDLE),
    PRE_INTAKE_CORAL_GROUND(
        ElevatorState.PRE_INTAKE_CORAL_GROUND,
        ArmState.INTAKE_CORAL_GROUND,
        IntakeState.INTAKE,
        RoutingState.INTAKE),
    INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ArmState.INTAKE_CORAL_GROUND,
        IntakeState.INTAKE,
        RoutingState.INTAKE),
    POST_INTAKE_CORAL_GROUND(
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
    BARGE(ElevatorState.BARGE, ArmState.BARGE),
    PROCESSOR(ElevatorState.PROCESSOR, ArmState.PROCESSOR),
    PRE_CLIMB(
        ElevatorState.IDLE,
        ArmState.CLIMB,
        IntakeState.INTAKE,
        RoutingState.IDLE,
        ClimberState.PRE_CLIMB);

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

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   */
  public record Transition(SuperState start, SuperState end, Trigger trigger) {}
  ;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private ArrayList<Transition> transitions = new ArrayList<>();

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

  public void addTransitions() {
    // ----Intake coral----
    transitions.add(
        new Transition(
            SuperState.IDLE,
            SuperState.PRE_INTAKE_CORAL_GROUND,
            Robot.intakeCoralReq.and(() -> !arm.hasCoral())));

    transitions.add(
        new Transition(
            SuperState.PRE_INTAKE_CORAL_GROUND,
            SuperState.INTAKE_CORAL_GROUND,
            new Trigger(() -> this.atExtension() && intake.hasCoral())));

    transitions.add(
        new Transition(
            SuperState.INTAKE_CORAL_GROUND,
            SuperState.POST_INTAKE_CORAL_GROUND,
            new Trigger(() -> arm.hasCoral())));

    transitions.add(
        new Transition(
            SuperState.POST_INTAKE_CORAL_GROUND,
            SuperState.READY_CORAL,
            new Trigger(() -> this.atExtension())));

    // ----L1----
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L1,
            new Trigger(() -> Robot.getCurrentCoralTarget() == ReefTarget.L1)
                .and(new Trigger(() -> arm.hasCoral()))
                .and(Robot.preScoreReq)));

    transitions.add(
        new Transition(
            SuperState.PRE_L1,
            SuperState.L1,
            new Trigger(() -> Robot.getCurrentCoralTarget() == ReefTarget.L1)
                .and(new Trigger(() -> arm.hasCoral()))
                .and(Robot.scoreReq)));

    transitions.add(
        new Transition(
            SuperState.L1,
            SuperState.POST_L1,
            new Trigger(
                () ->
                    !arm.hasCoral()
                        && !Robot.scoreReq
                            .getAsBoolean()))); // also might need to check that it's backed away
    // from the reef?

    transitions.add(
        new Transition(
            SuperState.POST_L1,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasCoral() && this.atExtension() && !Robot.scoreReq.getAsBoolean())));

    // ----L2----
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L2,
            new Trigger(() -> Robot.getCurrentCoralTarget() == ReefTarget.L2)
                .and(Robot.preScoreReq)));

    transitions.add(
        new Transition(
            SuperState.PRE_L2,
            SuperState.L2,
            new Trigger(() -> this.atExtension()).and(Robot.scoreReq)));

    transitions.add(
        new Transition(
            SuperState.L2,
            SuperState.POST_L2,
            new Trigger(
                () ->
                    !Robot.scoreReq
                        .getAsBoolean()))); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    transitions.add(
        new Transition(
            SuperState.POST_L2,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasCoral() && this.atExtension() && !Robot.scoreReq.getAsBoolean())));

    // ----L3----
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L3,
            new Trigger(() -> Robot.getCurrentCoralTarget() == ReefTarget.L3)
                .and(Robot.preScoreReq)));

    transitions.add(
        new Transition(
            SuperState.PRE_L3,
            SuperState.L3,
            new Trigger(() -> this.atExtension()).and(Robot.scoreReq)));

    transitions.add(
        new Transition(
            SuperState.L3,
            SuperState.POST_L3,
            new Trigger(
                () ->
                    !Robot.scoreReq
                        .getAsBoolean()))); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    transitions.add(
        new Transition(
            SuperState.POST_L3,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasCoral() && this.atExtension() && !Robot.scoreReq.getAsBoolean())));

    // ----L4----
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L4,
            new Trigger(() -> Robot.getCurrentCoralTarget() == ReefTarget.L4)
                .and(Robot.preScoreReq)));

    transitions.add(
        new Transition(
            SuperState.PRE_L4,
            SuperState.L4,
            new Trigger(() -> this.atExtension()).and(Robot.scoreReq)));

    transitions.add(
        new Transition(
            SuperState.L4,
            SuperState.POST_L4,
            new Trigger(() -> !Robot.scoreReq.getAsBoolean())
                .and(
                    () -> !arm.hasCoral()))); // also might need to check that it HASN'T backed away
    // from the reef?
    // also my gutfeel is that it should check the beambreak here but if it's on the pole it'll
    // still pick up on the coral?

    transitions.add(
        new Transition(
            SuperState.POST_L4,
            SuperState.IDLE,
            new Trigger(
                () -> !arm.hasCoral() && this.atExtension() && !Robot.scoreReq.getAsBoolean())));

    // maps triggers to the transitions
    for (Transition t : transitions) {
      t.trigger().and(new Trigger(() -> state == t.start)).onTrue(changeStateTo(t.end));
    }
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
