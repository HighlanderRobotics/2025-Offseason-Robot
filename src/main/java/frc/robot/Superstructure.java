// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.IntakeSubsystem.IntakeState;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.shoulder.ShoulderSubsystem.ShoulderState;

public class Superstructure{

  /**We should have a state for every single "pose" the robot will hit. Hopefully we can get named positions set up in cad to make this easier?
   * */
  public enum SuperState {
    IDLE(ElevatorState.IDLE, ShoulderState.IDLE, IntakeState.IDLE);

    public final ElevatorState elevatorState;
    public final ShoulderState shoulderState;
    public final IntakeState intakeState;

    private SuperState(
        ElevatorState elevatorState, ShoulderState shoulderState, IntakeState intakeState) {
      this.elevatorState = elevatorState;
      this.shoulderState = shoulderState;
      this.intakeState = intakeState;
    }
  }

    /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd additional cmd to be run while transitioning between the start and end states. 
   * Can be Commands.none() if nothing is needed beyond moving the subsystems
   */
  public record Transition(SuperState start, SuperState end, Trigger trigger, Command cmd) {};

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private ArrayList<Transition> transitions = new ArrayList<>();

  private Timer stateTimer = new Timer();

  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final IntakeSubsystem intake;

  /** Creates a new Superstructure. */
  public Superstructure(
    ElevatorSubsystem elevator,
      ShoulderSubsystem shoulder,
      IntakeSubsystem intake
  ) {
    this.elevator = elevator;
    this.shoulder = shoulder;
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
    //TODO
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
      shoulder.setState(state.shoulderState);
      intake.setState(state.intakeState);
  }
}
