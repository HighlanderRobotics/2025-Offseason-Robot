// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class RollerIOSim implements RollerIO {

  // for my own sanity i'm going to pretend there's only ever one motor
  private final DCMotorSim sim;

  public RollerIOSim(double jKgMetersSquared, double gearRatio) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), jKgMetersSquared, gearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    sim.update(0.02);
    inputs.voltage = sim.getInputVoltage();
    inputs.angularVelocityRPS = sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }
}
