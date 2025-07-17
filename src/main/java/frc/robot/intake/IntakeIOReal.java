// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

  private final TalonFX pivot;
  private final CANrange canrange;
  private final CANcoder cancoder;

  private final StatusSignal<Boolean> hasCoral;
  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Angle> cancoderPositionRotations;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<AngularVelocity> angularVelocityRPS;

  public IntakeIOReal() {
    pivot = new TalonFX(0, "*");
    canrange = new CANrange(0, "*");
    cancoder = new CANcoder(0, "*");

    hasCoral = canrange.getIsDetected();
    motorPositionRotations = pivot.getPosition();
    cancoderPositionRotations = cancoder.getAbsolutePosition();
    appliedVoltage = pivot.getMotorVoltage();
    angularVelocityRPS = pivot.getVelocity();

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // TODO PID things should go here
    // and other configs

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    final CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();

    cancoder.getConfigurator().apply(cancoderConfig);
    pivot.getConfigurator().apply(motorConfig);
    canrange.getConfigurator().apply(canrangeConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setVoltage(double voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void setMotorPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotorPosition'");
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
  }

  @Override
  public boolean hasCoral() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'hasCoral'");
  }
}
