package frc.robot.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal implements RollerIO {
  private final TalonFX rollerMotor;
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> motorTemperatureCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public RollerIOReal(int motorID, TalonFXConfiguration config) {
    rollerMotor = new TalonFX(motorID, "*");

    angularVelocityRotsPerSec = rollerMotor.getVelocity();
    supplyCurrentAmps = rollerMotor.getSupplyCurrent();
    appliedVoltage = rollerMotor.getMotorVoltage();
    statorCurrentAmps = rollerMotor.getStatorCurrent();
    motorTemperatureCelsius = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        motorTemperatureCelsius);

    rollerMotor.getConfigurator().apply(config);
    rollerMotor.optimizeBusUtilization();
  }

  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        supplyCurrentAmps,
        appliedVoltage,
        statorCurrentAmps,
        motorTemperatureCelsius);

    inputs.velocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.motorTemperatureCelsius = motorTemperatureCelsius.getValueAsDouble();
  }

  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double velocityRPS) {
    rollerMotor.setControl(velocityVoltage.withVelocity(velocityRPS));
  }
}
