package frc.robot.swerve.module;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.common.collect.ImmutableSet;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.module.Module.ModuleConstants;
import frc.robot.swerve.odometry.PhoenixOdometryThread;
import frc.robot.swerve.odometry.PhoenixOdometryThread.Registration;
import frc.robot.swerve.odometry.PhoenixOdometryThread.SignalType;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ModuleIOReal implements ModuleIO {
  private final ModuleConstants constants;

  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Status signals
  // For drive
  private final BaseStatusSignal drivePosition;
  private final BaseStatusSignal driveVelocity;
  private final BaseStatusSignal driveTemp;
  private final BaseStatusSignal driveSupplyCurrent;
  private final BaseStatusSignal driveStatorCurrent;
  private final BaseStatusSignal driveAppliedVolts;

  // For turn
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
  private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  public ModuleIOReal(ModuleConstants moduleConstants) {
    this.constants = moduleConstants;

    // Initialize hardware
    driveTalon = new TalonFX(constants.driveID(), "*");
    turnTalon = new TalonFX(constants.turnID(), "*");
    cancoder = new CANcoder(constants.cancoderID(), "*");

    // Configure hardware
    driveTalon.getConfigurator().apply(SwerveSubsystem.SWERVE_CONSTANTS.getDriveConfiguration());
    turnTalon
        .getConfigurator()
        .apply(SwerveSubsystem.SWERVE_CONSTANTS.getTurnConfiguration(constants.cancoderID()));

    cancoder
        .getConfigurator()
        .apply(
            SwerveSubsystem.SWERVE_CONSTANTS.getCancoderConfiguration(constants.cancoderOffset()));

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

    // Update the signals not updated by the odo thread at 50 hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveTemp,
        driveAppliedVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        cancoderAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent);
    // Register the signals to the odo thread
    PhoenixOdometryThread.getInstance()
        .registerSignals(
            new Registration(
                driveTalon,
                Optional.of(moduleConstants),
                SignalType.DRIVE,
                ImmutableSet.of(drivePosition)),
            new Registration(
                turnTalon,
                Optional.of(moduleConstants),
                SignalType.TURN,
                ImmutableSet.of(turnPosition)));
    // Still need to tell the motors to update these faster, even though we've registered them as
    // signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    Logger.recordOutput(
        "Module" + constants.prefix() + "refresh status code",
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
            turnTemp));

    inputs.prefix = constants.prefix();

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveTempC = driveTemp.getValueAsDouble();
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.cancoderAbsolutePosition =
        Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
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
  public void setDriveVelocitySetpoint(double setpointMetersPerSecond) {
    driveTalon.setControl(driveVelocityControl.withVelocity(setpointMetersPerSecond));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setTurnPositionSetpoint(Rotation2d setpoint) {
    turnTalon.setControl(turnPID.withPosition(setpoint.getRotations()));
  }
}
