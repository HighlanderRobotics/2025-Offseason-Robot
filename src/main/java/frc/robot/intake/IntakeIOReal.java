package frc.robot.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Angle> motorPositionRotations;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);

  public IntakeIOReal() {
    motor = new TalonFX(14, "*");

    angularVelocityRotsPerSec = motor.getVelocity();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();
    appliedVoltage = motor.getMotorVoltage();
    motorPositionRotations = motor.getPosition();

    // TODO PUT IN ACTUAL CONFIGS
    final var motorConfig = new TalonFXConfiguration();

    // config for arm/pivor motor
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kG = 0.0;
    motorConfig.Slot0.kS = 0.0;
    motorConfig.Slot0.kP = 0.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = 10; // Example ratio, adjust

    motor.getConfigurator().apply(motorConfig);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocityRotsPerSec,
        motorPositionRotations,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage);

    inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
  }
}
