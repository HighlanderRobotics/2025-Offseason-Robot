// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOReal implements ArmIO {
  private final TalonFX pivot;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Angle> cancoderPositionRotations;
  private final StatusSignal<Voltage> appliedPivotVoltage;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC positionTorque =
      new MotionMagicTorqueCurrentFOC(0.0); // TODO torque current or duty cycle

  public ArmIOReal() {
    pivot = new TalonFX(0, "*");
    cancoder = new CANcoder(0, "*");

    motorPositionRotations = pivot.getPosition();
    cancoderPositionRotations = cancoder.getAbsolutePosition();
    appliedPivotVoltage = pivot.getMotorVoltage();

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // TODO PID things should go here
    // and other configs

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoder.getConfigurator().apply(cancoderConfig);
    pivot.getConfigurator().apply(motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPositionRotations, cancoderPositionRotations, appliedPivotVoltage);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPositionRotations, cancoderPositionRotations, appliedPivotVoltage);

    inputs.motorPosition = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.cancoderPosition =
        Rotation2d.fromRotations(cancoderPositionRotations.getValueAsDouble());
    inputs.pivotVoltage = appliedPivotVoltage.getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivot.setControl(voltageOut.withOutput(voltage));
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
