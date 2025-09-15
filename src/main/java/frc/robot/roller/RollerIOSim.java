package frc.robot.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim motorSim;
  private final SimpleMotorFeedforward feedforward;
  private final ProfiledPIDController pid;

  private double appliedVolts = 0.0;

  public RollerIOSim(
      double jKgMetersSquared,
      double gearRatio,
      SimpleMotorFeedforward feedforward,
      ProfiledPIDController pid) {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), jKgMetersSquared, gearRatio),
            DCMotor.getKrakenX60Foc(1));
    this.feedforward = feedforward;
    this.pid = pid;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    motorSim.update(0.02);
    inputs.statorCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = 0.0;
  }

  @Override
  public void setRollerVoltage(double voltage) {
    appliedVolts = voltage;
    motorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setRollerVelocity(double velocityRPS) {
    setRollerVoltage(
        feedforward.calculate(velocityRPS)
            + pid.calculate(motorSim.getAngularVelocityRPM() / 60, velocityRPS));
  }
}
