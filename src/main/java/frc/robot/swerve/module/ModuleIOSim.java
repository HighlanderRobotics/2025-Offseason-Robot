package frc.robot.swerve.module;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utils.MaplePhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOSim implements ModuleIO {

  private final Module.ModuleConstants constants;

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
  private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVoltage turnPID = new MotionMagicVoltage(0.0);

  private final SwerveModuleSimulation simulation;

  public ModuleIOSim(Module.ModuleConstants constants, SwerveModuleSimulation simulation) {
    this.constants = constants;

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

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        driveVelocity,
        driveTemp,
        driveAppliedVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        cancoderAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnStatorCurrent,
        turnSupplyCurrent);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    // Setup this modules simulation (lowk the only difference between Sim and Real except for not
    // using Async odo)
    this.simulation = simulation;
    this.simulation.useDriveMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerSim(driveTalon, true));
    this.simulation.useSteerMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            turnTalon,
            SwerveSubsystem.SWERVE_CONSTANTS.getTurnMotorInverted(),
            cancoder,
            false,
            Angle.ofBaseUnits(constants.cancoderOffset().getRadians(), Radian)));
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
