// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
  private final TalonFX pivot;

  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Voltage> appliedPivotVoltage;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public ClimberIOReal() {
    pivot = new TalonFX(0, "*");

    motorPositionRotations = pivot.getPosition();
    appliedPivotVoltage = pivot.getMotorVoltage();

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // TODO PID things should go here
    // and other configs
    pivot.getConfigurator().apply(motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorPositionRotations, appliedPivotVoltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPositionRotations, appliedPivotVoltage);

    inputs.positionRotations =
        Rotation2d.fromRotations(
            motorPositionRotations
                .getValueAsDouble()); // TODO not sure how this works because there's the winch
    // thing
    inputs.pivotVoltage = appliedPivotVoltage.getValueAsDouble();
  }

  @Override
  // I lowkey really do not know how to do this because doesn't the diameter of the spool change?
  public void setPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivot.setControl(voltageOut.withOutput(voltage));
  }
}
