package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double TOLERANCE_DEGREES = 5.0;

  // TODO : change these values to the real ones
  public enum ClimberState {
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    // climbing
    PRE_CLIMB(Rotation2d.fromDegrees(0), 0.0),
    CLIMB(Rotation2d.fromDegrees(20), 0.0);

    public final Rotation2d position;
    public final double volts;

    ClimberState(Rotation2d position, double volts) {
      this.position = position;
      this.volts = volts;
    }
  }

  public ClimberSubsystem(RollerIO rollerIO, PivotIO pivotIO, String name) {
    super(rollerIO, pivotIO, name);
  }

  @AutoLogOutput(key = "Climber/State")
  private ClimberState state = ClimberState.IDLE;

  public void setState(ClimberState state) {
    this.state = state;
  }

  public ClimberState getState() {
    return state;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command setStateAngleVoltage(Supplier<ClimberState> stateSupplier) {
    return this.run(
        () -> {
          ClimberState state = stateSupplier.get();
          setPivotAngle(() -> state.position);
          runRollerVoltage(() -> state.volts);
        });
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public boolean atExtension() {
    return isNearAngle(super.getAngle());
  }
}
