package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmSubsystem.ArmState;
import frc.robot.canrange.CANrangeIO;
import frc.robot.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  private final CANrangeIO frontCanrangeIO;
  private final CANrangeIO rearCanrangeIO;
  private final CANrangeIOInputsAutoLogged frontCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final CANrangeIOInputsAutoLogged rearCanrangeInputs = new CANrangeIOInputsAutoLogged();

  public IntakeSubsystem(
      RollerIO rollerIO,
      PivotIO pivotIO,
      CANrangeIO frontCanrangeIO,
      CANrangeIO rearCanrangeIO,
      String name) {
    super(rollerIO, pivotIO, name);
    this.frontCanrangeIO = frontCanrangeIO;
    this.rearCanrangeIO = rearCanrangeIO;
  }

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public void setState(IntakeState state) {
    this.state = state;
  }

  // TODO : change these values to the real ones
  public enum IntakeState {
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

    IntakeState(Rotation2d position, double volts) {
      this.position = position;
      this.volts = volts;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    frontCanrangeIO.updateInputs(frontCanrangeInputs);
    Logger.processInputs("Intake/frontCanrange", frontCanrangeInputs);
    rearCanrangeIO.updateInputs(rearCanrangeInputs);
    Logger.processInputs("Intake/rearCanrange", rearCanrangeInputs);
  }

  public double getfrontCanrangePosition() {
    return frontCanrangeInputs.distanceCm;
  }

  public double getRearCanrangePosition() {
    return rearCanrangeInputs.distanceCm;
  }

  @AutoLogOutput(key = "Intake/HasGamePiece")
  public boolean hasGamePiece() {
    return getfrontCanrangePosition() < 10.0 || getRearCanrangePosition() < 10.0;
  }

  public Command zeroIntake() {
    return Commands.run(() -> setPivotVoltage(-2.0))
        .until(() -> currentFilterValue > 10.0)
        .andThen(
            Commands.runOnce(
                () -> {
                  zeroPivot();
                  setPivotVoltage(0.0);
                }));
  }

  public Command setStateAngleAndVoltage(ArmState state) {
    return this.runOnce(
        () -> {
          setPivotAngle(state.position);
          runRollerVoltage(state.volts);
        });
  }
}
