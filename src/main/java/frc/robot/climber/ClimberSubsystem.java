package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (45.0 / 16.0);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double LENGTH_METERS = 0.179;
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  // TODO tune
  public static final double KP = 0.2;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KG = 0.1;
  public static final double KV = 0.1;
  public static final double jKgMetersSquared = 0.01;
  public static final double TOLERANCE_DEGREES = 5.0;
  public static final Rotation2d CLIMB_EXTENSION_DEGREES = Rotation2d.fromDegrees(70);

  public enum ClimberState {
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    // climbing
    // TODO climber stuff is very tbd rn
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

  public Command setStateAngleVoltage() {
    return setPivotAndRollers(() -> state.position, () -> state.volts);
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public boolean atClimbExtension() {
    return isNearAngle(CLIMB_EXTENSION_DEGREES);
  }

  public Command retract() {
    return this.run(() -> setPivotVoltage(() -> -8.0));
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotIO.resetEncoder(Rotation2d.kZero));
  }
}
