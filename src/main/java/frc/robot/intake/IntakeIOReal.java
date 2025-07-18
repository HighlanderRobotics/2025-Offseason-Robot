// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

  private final TalonFX pivot;
  private final CANrange canrange;
  private final CANcoder cancoder;

  private final StatusSignal<Boolean> canrangeTriggered;
  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final MotionMagicTorqueCurrentFOC positionTorque =
      new MotionMagicTorqueCurrentFOC(0.0); // TODO torque current or duty cycle

  // TODO i'm not sure if L1 got cut or not
  public IntakeIOReal() {
    pivot = new TalonFX(0, "*");
    canrange = new CANrange(0, "*");
    cancoder = new CANcoder(0, "*");

    canrangeTriggered = canrange.getIsDetected();
    motorPositionRotations = pivot.getPosition();
    cancoderPositionRotations = cancoder.getAbsolutePosition();

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // TODO PID things should go here
    // and other configs

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    final CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();

    cancoder.getConfigurator().apply(cancoderConfig);
    pivot.getConfigurator().apply(motorConfig);
    canrange.getConfigurator().apply(canrangeConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, canrangeTriggered, motorPositionRotations, cancoderPositionRotations);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        canrangeTriggered, motorPositionRotations, cancoderPositionRotations);

    inputs.motorPosition = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.cancoderPosition =
        Rotation2d.fromRotations(cancoderPositionRotations.getValueAsDouble());
    inputs.canrange = canrangeTriggered.getValue();
  }

  @Override
  public void setPivotAngle(Rotation2d position) {
    pivot.setControl(positionTorque.withPosition(position.getRotations()));
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    cancoder.setPosition(position.getRotations());
  }
}
