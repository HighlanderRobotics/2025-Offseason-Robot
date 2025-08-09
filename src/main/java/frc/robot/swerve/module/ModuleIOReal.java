package frc.robot.swerve.module;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.swerve.module.Module.ModuleConstants;

public class ModuleIOReal implements ModuleIO {
  private final ModuleConstants constants;

  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Status signals
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveTemp;
  private final BaseStatusSignal driveSupplyCurrent;
  private final BaseStatusSignal driveStatorCurrent;
  private final BaseStatusSignal driveAppliedVolts;

  private final BaseStatusSignal cancoderAbsolutePosition;
  private final BaseStatusSignal turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final BaseStatusSignal turnTemp;
  private final BaseStatusSignal turnSupplyCurrent;
  private final BaseStatusSignal turnStatorCurrent;
  private final BaseStatusSignal turnAppliedVolts;

  // Control modes
  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);


  public ModuleIOReal(ModuleConstants moduleConstants) {
    this.constants = moduleConstants;

    // Initialize hardware
    driveTalon = new TalonFX(constants.driveID(), "*");
    turnTalon = new TalonFX(constants.turnID(), "*");
    cancoder = new CANcoder(constants.cancoderID(), "*");

    // Initialize status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveTemp = driveTalon.getDeviceTemp();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveStatorCurrent = driveTalon.getStatorCurrent();
    driveAppliedVolts = driveTalon.getMotorVoltage();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnTemp = turnTalon.getDeviceTemp();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnStatorCurrent = turnTalon.getStatorCurrent();
    turnAppliedVolts = turnTalon.getMotorVoltage();

    // TODO: USE AN ODOMETRY THREAD FOR SOME OF THESE
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition, driveVelocity, driveTemp, driveAppliedVolts, driveStatorCurrent, driveSupplyCurrent, cancoderAbsolutePosition, turnPosition, turnVelocity, turnAppliedVolts, turnStatorCurrent, turnSupplyCurrent);
    
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        driveTemp,
        cancoderAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent,
        turnTemp);
    
    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveTempC = driveTemp.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = turnVelocity.getValue().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnTempC = turnTemp.getValueAsDouble();
    inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(double volts, boolean withFoc) {
    driveTalon.setControl(driveVoltage.withOutput(volts).withEnableFOC(withFoc));
  }

  @Override
  public void setDriveSetpoint(double setpointMetersPerSecond) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveSetpoint'");
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setTurnSetpoint(Rotation2d setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnSetpoint'");
  }
}
