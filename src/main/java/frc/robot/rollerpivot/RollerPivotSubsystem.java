package frc.robot.rollerpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.pivot.PivotIO;
import frc.robot.pivot.PivotIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RollerPivotSubsystem extends SubsystemBase {
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  protected final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  protected final RollerIO rollerIO;
  protected final PivotIO pivotIO;
  private final String name;

  private LinearFilter currentFilter = LinearFilter.movingAverage(10);
  protected double currentFilterValue = 0.0;

  public RollerPivotSubsystem(RollerIO rollerIO, PivotIO pivotIO, String name) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.name = name;
  }

  protected void runRollerVoltage(double volts) {
    rollerIO.setRollerVoltage(volts);
  }

  protected void runRollerVelocity(double velocityRPS) {
    rollerIO.setRollerVelocity(velocityRPS);
  }

  protected void setPivotAngle(Rotation2d target) {
    Logger.recordOutput(name + "/Pivot Setpoint", target);
    pivotIO.setMotorPosition(target);
  }

  public Command setPivotVoltage(DoubleSupplier volts) {
    return this.run(() -> pivotIO.setMotorVoltage(volts.getAsDouble()));
  }

  public Rotation2d getPivotAngle() {
    return pivotInputs.position;
  }

  public double getPivotVoltage() {
    return pivotInputs.appliedVoltage;
  }

  public boolean isNear(Rotation2d target, double tolerance) {
    return MathUtil.isNear(target.getDegrees(), getPivotAngle().getDegrees(), tolerance);
  }

  public Command zeroPivot(Supplier<Rotation2d> rotations) {
    return this.runOnce(() -> pivotIO.resetEncoder(rotations.get()));
  }

  public double getFilteredStatorCurrentAmps() {
    return currentFilterValue;
  }

  // this CANNOT be correct LMAO
  public Command setPivotAndRollers(Rotation2d pivotAngle, double rollerVelocity) {
    // Command cmd =
    //     Commands.parallel(
    //         Commands.runOnce(() -> setPivotAngle(pivotAngle)),
    //         Commands.run(() -> runRollerVoltage(rollerVoltage)));
    // cmd.addRequirements(this);
    // return cmd;
    return this.run(
        () -> {
          Logger.recordOutput(name + "/Pivot Setpoint", pivotAngle);
          Logger.recordOutput(name + "/Rollers Setpoint", rollerVelocity);
          setPivotAngle(pivotAngle);
          runRollerVelocity(rollerVelocity);
        });
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs(name + "/Pivot", pivotInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(name + "/Roller", rollerInputs);

    currentFilterValue = currentFilter.calculate(rollerInputs.statorCurrentAmps);
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(name + "/Filtered Current", currentFilterValue);
  }
}
