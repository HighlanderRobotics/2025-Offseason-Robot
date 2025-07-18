// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;

/** Add your docs here. */
public class RollerIOReal implements RollerIO {
  private final ArrayList<TalonFX> motors = new ArrayList<>(); // TODO  what am i doing

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;

  public RollerIOReal(int[] ids, final TalonFXConfiguration config, boolean opposeLeader) {
    for (int i : ids) {
      motors.add(new TalonFX(i, "*"));
      motors.get(i).getConfigurator().apply(config);
    }
    for (int i = 1; i < motors.size(); i++) {
      motors.get(i).setControl(new Follower(motors.get(0).getDeviceID(), opposeLeader)); // bro
    }

    velocity = motors.get(0).getVelocity();
    voltage = motors.get(0).getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, voltage);

    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  @Override
  public void updateInputs(RollerIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(velocity, voltage);
    inputs.voltage = voltage.getValueAsDouble();
    inputs.angularVelocityRPS = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motors.get(0).setControl(voltageOut.withOutput(voltage));
  }
}
