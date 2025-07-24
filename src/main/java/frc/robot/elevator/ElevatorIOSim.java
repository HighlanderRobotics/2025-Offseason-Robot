// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorSubsystem.GEAR_RATIO,
          6.518 + 4.719,
          ElevatorSubsystem.DRUM_RADIUS_METERS,
          0.0,
          ElevatorSubsystem.MAX_EXTENSION_METERS,
          true,
          0);

  private double volts = 0.0;

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          40.0,
          0.0,
          0.1,
          new Constraints(
              5.0,
              10.0)); // Magic Numbers (idk if these are supposed to be the same as the irl numbers)
  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.0,
          0.06,
          (DCMotor.getKrakenX60Foc(1).KvRadPerSecPerVolt * ElevatorSubsystem.DRUM_RADIUS_METERS)
              / ElevatorSubsystem.GEAR_RATIO); // magic

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.positionMeters = sim.getPositionMeters();
    inputs.voltage = volts;
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
  }

  @Override
  public void setPosition(double positionMeters) {
    volts =
        pid.calculate(sim.getPositionMeters(), positionMeters)
            + ff.calculate(pid.getSetpoint().velocity);
    sim.setInputVoltage(volts);
  }
}
