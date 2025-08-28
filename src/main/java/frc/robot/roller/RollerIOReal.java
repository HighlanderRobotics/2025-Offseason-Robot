package frc.robot.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOReal implements RollerIO {
  private final TalonFX rollerMotor;

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  // private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);

  public RollerIOReal() {
    rollerMotor = new TalonFX(15, "*"); // Replace with actual ID

    angularVelocityRotsPerSec = rollerMotor.getVelocity();
    supplyCurrentAmps = rollerMotor.getSupplyCurrent();
    appliedVoltage = rollerMotor.getMotorVoltage();
    statorCurrentAmps = rollerMotor.getStatorCurrent();

    final var rollerMotorConfig = new TalonFXConfiguration();

    rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerMotorConfig);
    rollerMotor.optimizeBusUtilization();
  }

  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec, supplyCurrentAmps, appliedVoltage, statorCurrentAmps);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
  }

  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }
}
