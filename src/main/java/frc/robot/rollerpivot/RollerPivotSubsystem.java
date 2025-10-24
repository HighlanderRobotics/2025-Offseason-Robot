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
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final RollerIO rollerIO;
  private final PivotIO pivotIO;
  private final String name;

  private LinearFilter currentFilter = LinearFilter.movingAverage(10);
  public double currentFilterValue = 0.0;

  public RollerPivotSubsystem(RollerIO rollerIO, PivotIO pivotIO, String name) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.name = name;
  }

  public Command runRollerVoltage(DoubleSupplier volts) {
    return this.runOnce(() -> rollerIO.setRollerVoltage(volts.getAsDouble()));
  }

  public Command setPivotAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          Logger.recordOutput(name + "/Pivot Setpoint", target.get());
          pivotIO.setMotorPosition(target.get());
        });
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

  public Command zeroPivot(double position) {
    return this.runOnce(() -> pivotIO.resetEncoder(position));
  }

  public double getFilteredStatorCurrentAmps() {
    return currentFilterValue;
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
