package frc.robot.rollerAndPivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pivot.PivotIO;
import frc.robot.roller.RollerIO;
import java.util.function.Supplier;

public class rollerPivotSubsystem extends SubsystemBase {
  private final RollerIO rollers;
  private final PivotIO pivot;
  private final String name;

  public rollerPivotSubsystem(RollerIO rollers, PivotIO pivot, String name) {
    this.rollers = rollers;
    this.pivot = pivot;
    this.name = name;
  }

  public Command runRollerVoltage(double volts) {
    return this.run(() -> rollers.setRollerVoltage(volts));
  }

  public Command setRollerVoltage(double volts) {
    return setRollerVoltage(volts);
  }

  public Command setTargetAngle(Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Command setTargetAngle(Supplier<Rotation2d> target) {
    return setTargetAngle(target);
  }
}
