package frc.robot.rollerpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.pivot.PivotIO;
import frc.robot.pivot.PivotIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;

import static edu.wpi.first.units.Units.Volts;

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

  // Sysid stuff
  private final SysIdRoutine rollerSysid;
  private final SysIdRoutine pivotSysid;

  public RollerPivotSubsystem(RollerIO rollerIO, PivotIO pivotIO, String name) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.name = name;

    rollerSysid = new SysIdRoutine(
        // Defualt for now ig
        // Ramp rate: 1 volt, step rate: 7 volts, timeout: 10 volts
        new Config(),
        new Mechanism((voltage) -> runRollerVoltage(voltage.in(Volts)), null, this)
      );
    pivotSysid = new SysIdRoutine(
      new Config(),
      new Mechanism((voltage) -> runRollerVoltage(voltage.in(Volts)), null, this));
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

  public Command setPivotAngleAndRollerVoltage(
      Supplier<Rotation2d> pivotTarget, DoubleSupplier rollerVoltage) {
    return this.run(
        () -> {
          Logger.recordOutput(name + "/Pivot Setpoint", pivotTarget.get());
          pivotIO.setMotorPosition(pivotTarget.get());
          Logger.recordOutput(name + "/Roller Voltage", rollerVoltage.getAsDouble());
          rollerIO.setRollerVoltage(rollerVoltage.getAsDouble());
        });
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
  // public Command setPivotAndRollers(Rotation2d pivotAngle, double rollerVelocity) {
  //   // Command cmd =
  //   //     Commands.parallel(
  //   //         Commands.runOnce(() -> setPivotAngle(pivotAngle)),
  //   //         Commands.run(() -> runRollerVoltage(rollerVoltage)));
  //   // cmd.addRequirements(this);
  //   // return cmd;
  //   return this.run(
  //       () -> {
  //         Logger.recordOutput(name + "/Pivot Setpoint", pivotAngle);
  //         Logger.recordOutput(name + "/Rollers Setpoint", rollerVelocity);
  //         setPivotAngle(pivotAngle);
  //         runRollerVelocity(rollerVelocity);
  //       });
  // }

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

  public Command runRollerSysid() {
    return Commands.sequence(
      // Run a quasistatic (w/o acceleration) sysid in each direction, and a dynamic (with acceleration) sysid
      rollerSysid.quasistatic(Direction.kForward),
      rollerSysid.quasistatic(Direction.kReverse),
      rollerSysid.dynamic(Direction.kForward),
      rollerSysid.dynamic(Direction.kReverse)
    );
  }

  public Command runPivotSysid() {
    return Commands.sequence(
      pivotSysid.quasistatic(Direction.kForward),
      pivotSysid.quasistatic(Direction.kReverse),
      pivotSysid.dynamic(Direction.kForward),
      pivotSysid.dynamic(Direction.kReverse)
    );
  }
}
