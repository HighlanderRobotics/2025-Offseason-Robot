package frc.robot.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO{
      // TODO: change to actual values
  private final SingleJointedArmSim armSim =
  new SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      ArmSubsystem.PIVOT_RATIO, //pivot ratio
      0.1,
      Units.inchesToMeters(20), //arm length
      ArmSubsystem.MIN_ANGLE.getRadians(), //min angle
      ArmSubsystem.MAX_ANGLE.getRadians(), //max angle 
      true,
      0.0);

private final ProfiledPIDController pivotPid =
  new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));
private final ArmFeedforward pivotFf = new ArmFeedforward(0.0, 0.0, 0.0); 

private double appliedVoltage = 0.0;

@Override
public void updateInputs(ArmIOInputs inputs) {
armSim.update(0.02);

inputs.angularVelocityRotsPerSec =
    RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);
inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
inputs.statorCurrentAmps = armSim.getCurrentDrawAmps();
inputs.supplyCurrentAmps = 0.0;
inputs.appliedVoltage = appliedVoltage;
}

@Override
public void setMotorVoltage(double voltage) {
appliedVoltage = voltage;
armSim.setInputVoltage(MathUtil.clamp(voltage, -20, 20));
}

@Override
public void setMotorPosition(Rotation2d targetPosition) {
setMotorVoltage(
    pivotPid.calculate(armSim.getAngleRads(), targetPosition.getRadians())
        + pivotFf.calculate(pivotPid.getSetpoint().position, pivotPid.getSetpoint().velocity));
}

@Override
public void setRollerVoltage(double voltage){
   //not gonna simulate the rollers
}

public void setResetSimState() {
armSim.setState(0.0, 0.0);
}
}
