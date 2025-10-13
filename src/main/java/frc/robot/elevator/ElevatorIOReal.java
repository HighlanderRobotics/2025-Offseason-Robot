package frc.robot.elevator;

// libraries
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
  // init motors
  private final TalonFX motor = new TalonFX(0, "*"); // put correct ID
  private final TalonFX follower = new TalonFX(1, "*"); // put correct ID

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torque = new TorqueCurrentFOC(0.0);
  private final MotionMagicExpoVoltage positionTorque =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> voltage = motor.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();

  // private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();

  public ElevatorIOReal() {
    // TODO elevator config
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO / (2 * Math.PI * ElevatorSubsystem.DRUM_RADIUS_METERS);
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = 0.43832;
    config.Slot0.kS = 1.1062;
    config.Slot0.kV = 1.9542;
    config.Slot0.kA = 0.26245;
    config.Slot0.kP = 69.925;
    config.Slot0.kD = 5.5908;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 8.0;
    config.MotionMagic.MotionMagicCruiseVelocity =
        (5500.0 / 60.0) / config.Feedback.SensorToMechanismRatio;

    config.MotionMagic.MotionMagicExpo_kV = 1.9542;
    config.MotionMagic.MotionMagicExpo_kA = 0.26245;

    motor.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    // set follower
    follower.setControl(new Follower(motor.getDeviceID(), true));

    // set current limits
    // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0, 1.0));
    // follower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 40.0,
    // 1.0));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, voltage, statorCurrent, supplyCurrent);
  }

  @Override
  public void updateInputs(final ElevatorIOInputsAutoLogged inputs) {
    // update inputs
    BaseStatusSignal.refreshAll(position, velocity, voltage, statorCurrent, supplyCurrent);
    inputs.positionMeters = position.getValueAsDouble();
    inputs.velocityMetersPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    // inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(final double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setPositionTarget(final double meters) {
    motor.setControl(positionTorque.withPosition(meters));
  }

  @Override
  public void setCurrent(final double amps) {
    motor.setControl(torque.withOutput(amps));
  }

  @Override
  public void resetEncoder(final double position) {
    motor.setPosition(position);
  }
}
