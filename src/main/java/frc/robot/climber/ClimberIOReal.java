package frc.robot.climber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOReal implements ClimberIO {
    private TalonFX pivot = new TalonFX(16, "*");
    private TalonFX roller = new TalonFX(17, "*");

    private StatusSignal<Angle> pivotPosition = pivot.getPosition();
    private StatusSignal<AngularVelocity> pivotVelocity = pivot.getVelocity();
    private StatusSignal<Voltage> pivotVoltage = pivot.getMotorVoltage();
    private StatusSignal<Current> pivotStatorCurrent = pivot.getStatorCurrent();
    private StatusSignal<Current> pivotSupplyCurrent = pivot.getSupplyCurrent();
    private StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();
    
    private StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity();
    private StatusSignal<Voltage> rollerVoltage = roller.getMotorVoltage();
    private StatusSignal<Current> rollerStatorCurrent = roller.getStatorCurrent();
    private StatusSignal<Current> rollerSupplyCurrent = roller.getSupplyCurrent();
    private StatusSignal<Temperature> rollerTemp = roller.getDeviceTemp();

    private MotionMagicVoltage pivotPositionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

    private VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);
    private VoltageOut rollerVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

    public ClimberIOReal() {
        // TODO: CONFIGS

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotPosition, pivotVelocity, pivotVoltage, pivotStatorCurrent, pivotSupplyCurrent, pivotTemp, rollerVelocity, rollerVoltage, rollerStatorCurrent, rollerSupplyCurrent, rollerTemp);
        pivot.optimizeBusUtilization();
        roller.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotVoltage, pivotStatorCurrent, pivotSupplyCurrent, pivotTemp, rollerVelocity, rollerVoltage, rollerStatorCurrent, rollerSupplyCurrent, rollerTemp);
        
        inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValue().in(Rotations));
        inputs.pivotVelocityRotPerSec = pivotVelocity.getValue().in(RotationsPerSecond);
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.getValueAsDouble();
        inputs.pivotTempC = pivotTemp.getValueAsDouble();

        inputs.rollerVelocityRotPerSec = rollerVelocity.getValue().in(RotationsPerSecond);
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerTempC = rollerTemp.getValueAsDouble();
    }

    @Override
    public void setPivotPosition(Rotation2d setpoint) {
        pivot.setControl(pivotPositionVoltage.withPosition(setpoint.getMeasure())); 
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivot.setControl(pivotVoltageOut.withOutput(volts));   
    }

    @Override
    public void setRollerVelocity(double rotationsPerSecond) {
        roller.setControl(rollerVelocityVoltage.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setRollerVoltage(double volts) {
        roller.setControl(rollerVoltageOut.withOutput(volts));
    }

}
