package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.cancoder.CANcoderIO;
import frc.robot.cancoder.CANcoderIOInputsAutoLogged;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double LENGTH_METERS = 0.659;
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  public static final double ROLLERS_RATIO = (44.0 / 16.0) * 23;

  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final double SENSOR_TO_MECH_RATIO = 0.0;
  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KG = 0.4;
  public static final double KV = 0.0;
  public static final double jKgMetersSquared = 0.01;
  // public static final double GAME_PIECE_CURRENT_THRESHOLD = 20.0;
  public static final double ALGAE_INTAKE_VOLTAGE = 8.0;
  public static final double CORAL_INTAKE_VOLTAGE = 5.0;
  public static final double ALGAE_CURRENT_THRESHOLD = 20.0;
  public static final double CORAL_CURRENT_THRESHOLD = 20.0;
  public static final double TOLERANCE_DEGREES = 10.0;

  public static final double CANCODER_OFFSET = 0.688;
  // this is because we want it to wrap around from 360 to 0 when it's vertical, which is what we've
  // decided is 0
  public static final double CANCODER_DISCONTINUITY_POINT = 0.688;

  private final CANcoderIO cancoderIO;
  private final CANcoderIOInputsAutoLogged cancoderInputs = new CANcoderIOInputsAutoLogged();

  public boolean hasAlgae = false;
  public boolean hasCoral = false;

  /**
   * 0 for position is vertical with the EE up. We consider the front of the robot to be the intake,
   * so left and right are based on that. Positive is counterclockwise from the robot's POV (which
   * is to the left). We're in real life!! use degrees. degrees -> Rotation2d gets handled in the
   * constructor Positive voltage is intaking, negative is outtaking. (TODO)
   */
  public enum ArmState {
    IDLE(0, 0.0),
    // coral
    HANDOFF(180, 5.0),
    READY_CORAL_ARM(0, 1.0),
    INTAKE_CORAL_STACK(100, 5.0),

    PRE_L2_RIGHT(-45, 1.0),
    SCORE_L2_RIGHT(-90, -10.0),
    PRE_L3_RIGHT(-45, 1.0),
    SCORE_L3_RIGHT(-90, -10.0),
    PRE_L4_RIGHT(-70, 1.0),
    SCORE_L4_RIGHT(-90, -10.0),

    PRE_L2_LEFT(45, 1.0),
    SCORE_L2_LEFT(90, -10.0),
    PRE_L3_LEFT(45, 1.0),
    SCORE_L3_LEFT(90, -10.0),
    PRE_L4_LEFT(70, 1.0),
    SCORE_L4_LEFT(90, -10.0),
    // algae
    INTAKE_ALGAE_REEF_RIGHT(-90, 10.0),
    INTAKE_ALGAE_REEF_LEFT(90, 10.0),
    INTAKE_ALGAE_GROUND(125, 15.0),
    INTAKE_ALGAE_STACK(90, 10.0),
    READY_ALGAE(0, 2.0),

    PRE_BARGE_RIGHT(-20, 4.0),
    SCORE_BARGE_RIGHT(-20, -10.0),

    PRE_BARGE_LEFT(20, 4.0),
    SCORE_BARGE_LEFT(20, -10.0),

    PRE_PROCESSOR(180, 0.0),
    SCORE_PROCESSOR(180, -10.0),
    // climbing
    PRE_CLIMB(180, 0.0),
    CLIMB(180, 0.0);

    public final Rotation2d position;
    public final double volts;

    private ArmState(double positionDegrees, double volts) {
      this.position = Rotation2d.fromDegrees(positionDegrees);
      this.volts = volts;
    }

    public Rotation2d getAngle() {
      return position;
    }

    public double getVolts() {
      return volts;
    }
  }

  public ArmSubsystem(RollerIO rollerIO, PivotIO pivotIO, CANcoderIO cancoderIO, String name) {
    super(rollerIO, pivotIO, name);
    this.cancoderIO = cancoderIO;
  }

  @AutoLogOutput(key = "Arm/State")
  private ArmState state = ArmState.IDLE;

  public void setState(ArmState state) {
    this.state = state;
  }

  public ArmState getState() {
    return state;
  }

  @Override
  public void periodic() {
    super.periodic();
    cancoderIO.updateInputs(cancoderInputs);
    Logger.processInputs("Arm/CANcoder", cancoderInputs);
  }

  public Rotation2d getCANcoderPosition() {
    return cancoderInputs.cancoderPosition;
  }

  public Command intakeAlgae() {
    return this.run(() -> runRollerVoltage(() -> ALGAE_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.runOnce(() -> hasAlgae = true));
  }

  public Command intakeCoral() {
    return this.run(() -> runRollerVoltage(() -> CORAL_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > CORAL_CURRENT_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.runOnce(() -> hasCoral = true));
  }

  // can it distinguish between coral and algae?
  @AutoLogOutput(key = "Arm/Has Game Piece")
  public boolean hasGamePiece() {
    // return (Math.abs(currentFilterValue) > CORAL_CURRENT_THRESHOLD ||
    // Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD);
    return hasAlgae || hasCoral;
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  // TODO this is definitely not Good
  public Command setStateAngleVoltage() {
    return setPivotAngle(() -> state.position)
        .andThen(runRollerVoltage(() -> state.volts))
        .repeatedly();
  }

  // TODO setSimCoral
  public void setSimCoral(boolean b) {}

  public Command rezeroFromEncoder() {
    return this.runOnce(() -> zeroPivot(cancoderInputs.cancoderPosition.getDegrees()));
  }
}
