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
  // public static final double GAME_PIECE_CURRENT_THRESHOLD = 20.0;
  public static final double ALGAE_INTAKE_VOLTAGE = 8.0;
  public static final double CORAL_INTAKE_VOLTAGE = 5.0;
  public static final double ALGAE_CURRENT_THRESHOLD = 20.0;
  public static final double CORAL_CURRENT_THRESHOLD = 20.0;

  private final CANcoderIO cancoderIO;
  private final CANcoderIOInputsAutoLogged cancoderInputs = new CANcoderIOInputsAutoLogged();

  @AutoLogOutput public boolean hasAlgae = false;
  @AutoLogOutput public boolean hasCoral = false;

  public ArmSubsystem(RollerIO rollerIO, PivotIO pivotIO, CANcoderIO cancoderIO, String name) {
    super(rollerIO, pivotIO, name);
    this.cancoderIO = cancoderIO;
  }

  @AutoLogOutput(key = "Arm/State")
  private ArmState state = ArmState.IDLE;

  public void setState(ArmState state) {
    this.state = state;
  }

  // TODO : change these values to the real ones
  public enum ArmState {
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

    ArmState(Rotation2d position, double volts) {
      this.position = position;
      this.volts = volts;
    }
  }

  public Rotation2d getCANcoderPosition() {
    return cancoderInputs.cancoderPosition;
  }

  public Command intakeAlgae() {
    return this.run(() -> runRollerVoltage(ALGAE_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.runOnce(() -> hasAlgae = true));
  }

  public Command intakeCoral() {
    return this.run(() -> runRollerVoltage(CORAL_INTAKE_VOLTAGE))
        .until(
            new Trigger(() -> Math.abs(currentFilterValue) > CORAL_CURRENT_THRESHOLD)
                .debounce(0.25))
        .andThen(Commands.runOnce(() -> hasCoral = true));
  }

  @AutoLogOutput(key = "Arm/Has GamePiece")
  public boolean hasGamePiece() {
    // return (Math.abs(currentFilterValue) > CORAL_CURRENT_THRESHOLD ||
    // Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD);
    return hasAlgae || hasCoral;
  }

  @Override
  public void periodic() {
    super.periodic();
    cancoderIO.updateInputs(cancoderInputs);
    Logger.processInputs("Arm/CANcoder", cancoderInputs);
  }

  public Command setStateAngleAndVoltage(ArmState state) {
    return this.runOnce(
        () -> {
          setPivotAngle(state.position);
          runRollerVoltage(state.volts);
        });
  }
}
