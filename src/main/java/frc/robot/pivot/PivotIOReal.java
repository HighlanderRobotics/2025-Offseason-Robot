package frc.robot.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOReal implements PivotIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Temperature> motorTemperatureCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public PivotIOReal(int motorID, TalonFXConfiguration config) {
    motor = new TalonFX(motorID, "*");

    angularVelocityRotsPerSec = motor.getVelocity();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    appliedVoltage = motor.getMotorVoltage();
    motorPositionRotations = motor.getPosition();
    motorTemperatureCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorPositionRotations,
        motorTemperatureCelsius);

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        motorPositionRotations,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorTemperatureCelsius);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.motorTemperatureCelsius = motorTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition, int slot) {
    // motor.setControl(motionMagic.withPosition(targetPosition.getRotations()).withSlot(slot));
    motor.setControl(positionVoltage.withPosition(targetPosition.getRotations()).withSlot(slot));
  }

  @Override
  public void resetEncoder(Rotation2d rotations) {
    motor.setPosition(rotations.getRotations());
  }
}
