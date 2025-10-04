// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.ArmSubsystem;
import frc.robot.arm.ArmSubsystem.ArmState;
import frc.robot.climber.ClimberSubsystem.ClimberState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.IntakeSubsystem.IntakeState;
import java.util.ArrayList;

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

    INTAKE_CORAL_STACK(ElevatorState.INTAKE_CORAL_STACK, ArmState.INTAKE_CORAL_STACK, IntakeState.CLIMB),

    READY_CORAL(ElevatorState.IDLE, ArmState.READY_CORAL, IntakeState.IDLE),
    
    PRE_L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.PRE_L1),
    L1(ElevatorState.IDLE, ArmState.IDLE, IntakeState.SCORE_L1),
    PRE_L2(ElevatorState.PRE_L2, ArmState.PRE_L2, IntakeState.IDLE),
    SCORE_L2(ElevatorState.L2, ArmState.SCORE_L2, IntakeState.IDLE),
    PRE_L3(ElevatorState.PRE_L3, ArmState.PRE_L3, IntakeState.IDLE),
    SCORE_L3(ElevatorState.L3, ArmState.SCORE_L3, IntakeState.IDLE),
    PRE_L4(ElevatorState.PRE_L4, ArmState.PRE_L4, IntakeState.IDLE),
    SCORE_L4(ElevatorState.L4, ArmState.SCORE_L4, IntakeState.IDLE),

    INTAKE_ALGAE_REEF_HIGH(
        ElevatorState.INTAKE_ALGAE_REEF_HIGH, ArmState.INTAKE_ALGAE_REEF, IntakeState.IDLE),
    INTAKE_ALGAE_REEF_LOW(
        ElevatorState.INTAKE_ALGAE_REEF_LOW, ArmState.INTAKE_ALGAE_REEF, IntakeState.IDLE),
    INTAKE_ALGAE_STACK(ElevatorState.INTAKE_ALGAE_STACK, ArmState.INTAKE_ALGAE_STACK, IntakeState.IDLE),
    INTAKE_ALGAE_GROUND(
        ElevatorState.INTAKE_ALGAE_GROUND, ArmState.INTAKE_ALGAE_GROUND, IntakeState.IDLE),

    READY_ALGAE(ElevatorState.READY_ALGAE, ArmState.READY_ALGAE, IntakeState.IDLE),
    
    PRE_BARGE(ElevatorState.BARGE, ArmState.PRE_BARGE, IntakeState.IDLE),
    SCORE_BARGE(ElevatorState.BARGE, ArmState.SCORE_BARGE, IntakeState.IDLE),
    
    PRE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.PRE_PROCESSOR, IntakeState.IDLE),
    SCORE_PROCESSOR(ElevatorState.PROCESSOR, ArmState.SCORE_PROCESSOR, IntakeState.IDLE),
    
    PRE_CLIMB(ElevatorState.PRE_CLIMB, ArmState.PRE_CLIMB, IntakeState.CLIMB),
    CLIMB(ElevatorState.CLIMB, ArmState.CLIMB, IntakeState.CLIMB);

    public final ElevatorState elevatorState;
    public final ArmState armState;
    public final IntakeState intakeState;
    public final ClimberState climberState;

    private SuperState(ElevatorState elevatorState, ArmState armState, IntakeState intakeState, ClimberState climberState) {
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
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd additional cmd to be run while transitioning between the start and end states. Can
   *     be Commands.none() if nothing is needed beyond moving the subsystems
   */
  public record Transition(SuperState start, SuperState end, Trigger trigger, Command cmd) {}
  ;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private ArrayList<Transition> transitions = new ArrayList<>();

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;

  /** Creates a new Superstructure. */
  public Superstructure(ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;

    addTransitions();
  }

  public void periodic() {
    for (Transition t : transitions) {
      if (state == t.start && t.trigger.getAsBoolean()) {
        forceState(t.end);
        t.cmd();
        return;
      }
    }
  }

  public void addTransitions() {
    // TODO
  }

  private Command forceState(SuperState nextState) {
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

  private void setSubstates() {
    elevator.setState(state.elevatorState);
    arm.setArmState(state.armState);
    intake.setState(state.intakeState);
  }
}
