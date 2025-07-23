// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Angle> position; // TODO why is this an angle???
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<AngularVelocity> velocity; // these units are really bothering me

  private final MotionMagicTorqueCurrentFOC positionTorque =
      new MotionMagicTorqueCurrentFOC(
          0.0); // TODO i still don't know which control request you would use when

  public ElevatorIOReal() {
    leader = new TalonFX(10, "*"); // TODO don't forget about device ids
    follower = new TalonFX(11, "*");

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO
            / (2
                * Math.PI
                * ElevatorSubsystem
                    .DRUM_RADIUS_METERS); // i hate not knowing where numbers come from

    position = leader.getPosition();
    appliedVoltage = leader.getMotorVoltage();
    velocity = leader.getVelocity();

    follower.setControl(new Follower(10, false));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVoltage, velocity);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, appliedVoltage, velocity);

    inputs.positionMeters = position.getValueAsDouble(); // what??
    inputs.voltage = appliedVoltage.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionMeters) {
    leader.setControl(positionTorque.withPosition(positionMeters));
  }
}
