package frc.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIOReal {

  @AutoLog
  public static class ElevatorIOInputs {
    public double leaderPositionMeters = 0.0;
    public double leaderVelocityMetersPerSec = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderVoltage = 0.0;
    public double leaderTempC = 0.0;

    public double followerPositionMeters = 0.0;
    public double followerVelocityMetersPerSec = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerVoltage = 0.0;
    public double followerTempC = 0.0;
  }

  protected TalonFX leader = new TalonFX(10, "*");
  protected TalonFX follower = new TalonFX(11, "*");

  // Conversion from angle to distance happens in sensor to mechanism ratio
  private final BaseStatusSignal leaderPositionMeters = leader.getPosition();
  private final BaseStatusSignal leaderVelocityMetersPerSec = leader.getVelocity();
  private final StatusSignal<Voltage> leaderVoltage = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Temperature> leaderTemp = leader.getDeviceTemp();

  private final BaseStatusSignal followerPositionMeters = follower.getPosition();
  private final BaseStatusSignal followerVelocityMetersPerSec = follower.getVelocity();
  private final StatusSignal<Voltage> followerVoltage = follower.getMotorVoltage();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Temperature> followerTemp = follower.getDeviceTemp();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private DynamicMotionMagicVoltage motionMagicVoltage;
  private TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0.0);

  // this is only for sim
  protected double positionSetpoint = 0.0;

  public ElevatorIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Converts angular motion to linear motion
    config.Feedback.SensorToMechanismRatio =
        ElevatorSubsystem.GEAR_RATIO / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS);

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kS = 0.24;
    config.Slot0.kG = 0.56;
    config.Slot0.kV = 0.6;
    config.Slot0.kP = 110.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.25;

    config.MotionMagic.MotionMagicAcceleration = ElevatorSubsystem.MAX_ACCELERATION;
    // This is what's set in sim but idk.
    config.MotionMagic.MotionMagicCruiseVelocity = 5.0;

    motionMagicVoltage =
        new DynamicMotionMagicVoltage(
                0.0,
                config.MotionMagic.MotionMagicCruiseVelocity,
                config.MotionMagic.MotionMagicAcceleration,
                100.0)
            .withEnableFOC(true);

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    follower.setControl(new Follower(leader.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPositionMeters,
        leaderVelocityMetersPerSec,
        leaderVoltage,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        leaderTemp,
        followerPositionMeters,
        followerVelocityMetersPerSec,
        followerVoltage,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionMeters,
        leaderVelocityMetersPerSec,
        leaderVoltage,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        leaderTemp,
        followerPositionMeters,
        followerVelocityMetersPerSec,
        followerVoltage,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);

    inputs.leaderPositionMeters = leaderPositionMeters.getValueAsDouble();
    inputs.leaderVelocityMetersPerSec = leaderVelocityMetersPerSec.getValueAsDouble();
    inputs.leaderVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTempC = leaderTemp.getValueAsDouble();

    inputs.followerPositionMeters = followerPositionMeters.getValueAsDouble();
    inputs.followerVelocityMetersPerSec = followerVelocityMetersPerSec.getValueAsDouble();
    inputs.followerVoltage = followerVoltage.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTempC = followerTemp.getValueAsDouble();
  }

  public void setVoltage(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  public void setCurrent(double amps) {
    leader.setControl(torqueCurrent.withOutput(amps));
  }

  public void setPositionSetpoint(double positionMeters, double acceleration) {
    positionSetpoint = positionMeters;
    leader.setControl(
        motionMagicVoltage.withPosition(positionMeters).withAcceleration(acceleration));
  }

  public void resetEncoder(double position) {
    leader.setPosition(position);
    follower.setPosition(position);
  }

  public void stop() {
    setVoltage(0.0);
  }
}
