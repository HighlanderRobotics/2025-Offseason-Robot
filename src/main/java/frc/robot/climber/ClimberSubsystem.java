package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);

  // TODO : change these values to the real ones
  public enum ClimberState {
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    // coral
    READY_CORAL(Rotation2d.fromDegrees(180), 0.0),
    PRE_INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(180), 0.0),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(180), 0.0),
    L1(Rotation2d.fromDegrees(100), 10.0),
    PRE_L2(Rotation2d.fromDegrees(45), 10.0),
    L2(Rotation2d.fromDegrees(60), 10.0),
    PRE_L3(Rotation2d.fromDegrees(45), 10.0),
    L3(Rotation2d.fromDegrees(70), 10.0),
    PRE_L4(Rotation2d.fromDegrees(60), 10.0),
    L4(Rotation2d.fromDegrees(90), 10.0),
    POST_L4(Rotation2d.fromDegrees(90), 0.0),
    // algae
    INTAKE_ALGAE_REEF_HIGH(Rotation2d.fromDegrees(90), 0.0),
    INTAKE_ALGAE_REEF_LOW(Rotation2d.fromDegrees(90), 0.0),
    INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(135), 0.0),
    BARGE(Rotation2d.fromDegrees(30), 0.0),
    READY_ALGAE(Rotation2d.fromDegrees(0), 0.0),
    PROCESSOR(Rotation2d.fromDegrees(90), 0.0),
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

  public Command setStateAngleVoltage(ClimberState state) {
    return this.runOnce(
        () -> {
          setPivotAngle(state.position);
          runRollerVoltage(state.volts);
        });
  }

  // TODO atExtension
  public boolean atExtension() {
    return true;
  }
}
