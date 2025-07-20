package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

public class ArmIOReal implements ArmIO {
      private final TalonFX motor = new TalonFX(14, "*");

  private final StatusSignal<AngularVelocity> angularVelocityRotsPerSec = motor.getVelocity();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
  private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
  private final StatusSignal<Angle> motorPositionRotations = motor.getPosition();

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        final var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Slot0.kV = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
    
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    
        BaseStatusSignal.refreshAll(angularVelocityRotsPerSec,
        supplyCurrentAmps, 
        appliedVoltage, 
        statorCurrentAmps);

        inputs.angularVelocityRotsPerSec = angularVelocityRotsPerSec.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.position = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    }

    @Override
    public void setMotorVoltage(double voltage){
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setMotorPosition(Rotation2d targetPosition){
        motor.setControl(motionMagic.withPosition(targetPosition.getRotations()));
    }

}
