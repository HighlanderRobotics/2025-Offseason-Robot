package frc.robot.pivot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim pivotSim;
  private final ProfiledPIDController pivotPid;
  private final ArmFeedforward pivotFf;

  public PivotIOSim(
      double minAngleRadians,
      double maxAngleRadians,
      double length,
      double maxVelocity,
      double maxAcceleration,
      TalonFXConfiguration config) {
    pivotSim =
        new SingleJointedArmSim(
            new DCMotor(12.0, 4.05, 275, 1.4, 7530.0 / 60.0, 1),
            config.Feedback.SensorToMechanismRatio,
            0.1,
            length,
            minAngleRadians,
            maxAngleRadians,
            true,
            0.0);

    pivotPid =
        new ProfiledPIDController(
            config.Slot0.kP,
            config.Slot0.kI,
            config.Slot0.kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    pivotFf = new ArmFeedforward(config.Slot0.kS, config.Slot0.kG, config.Slot0.kV);
  }

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(0.02);

    inputs.angularVelocityRotsPerSec =
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.position = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.statorCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    appliedVoltage = voltage;
    pivotSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition, int slot) {
    setMotorVoltage(
        pivotPid.calculate(pivotSim.getAngleRads(), targetPosition.getRadians())
            + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
  }

  @Override
  public void resetEncoder(Rotation2d rotations) {}
}
