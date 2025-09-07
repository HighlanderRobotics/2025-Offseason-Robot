package frc.robot.Pivot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim pivotSim;

  // TODO: change to actual values
  public PivotIOSim(
      double PIVOT_RATIO, double MIN_ANGLE_RADIANS, double MAX_ANGLE_RADIANS, double length) {
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            PIVOT_RATIO,
            0.1,
            length,
            MIN_ANGLE_RADIANS, // min angle
            MAX_ANGLE_RADIANS, // max angle
            true,
            0.0);
  }

  private final ProfiledPIDController pivotPid =
      // TODO tune these values
      new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));
  private final ArmFeedforward pivotFf = new ArmFeedforward(0.2, 0.2, 0.2);

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
    pivotSim.setInputVoltage(MathUtil.clamp(voltage, -20, 20));
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    setMotorVoltage(
        pivotPid.calculate(pivotSim.getAngleRads(), targetPosition.getRadians())
            + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
  }
}
