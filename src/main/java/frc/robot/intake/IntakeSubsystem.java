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
  public static final double TOLERANCE_DEGREES = 15.0;

  private final CANrangeIO frontCanrangeIO;
  private final CANrangeIO rearCanrangeIO;
  private final CANrangeIOInputsAutoLogged leftCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final CANrangeIOInputsAutoLogged rightCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final double ZEROING_OFFSET = -10;
  private boolean intakeHasZeroed = false;
  private Rotation2d currentPosition = new Rotation2d();

  // TODO : change these values to the real ones
  public enum IntakeState {
    IDLE(130, 0.0),
    INTAKE_CORAL(0, 10.0),
    READY_CORAL_INTAKE(130, 1.0),
    HANDOFF(130, -5.0),
    PRE_L1(90, 1.0),
    SCORE_L1(9, -5.0),
    CLIMB(0, 0.0);

    public final Rotation2d position;
    public final double volts;

    private IntakeState(double positionDegrees, double volts) {
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

  // private boolean isIntakingState(IntakeState state) {
  //   return state == IntakeState.INTAKE_ALGAE_GROUND
  //       || state == IntakeState.INTAKE_ALGAE_REEF_HIGH
  //       || state == IntakeState.INTAKE_ALGAE_REEF_LOW
  //       || state == IntakeState.INTAKE_CORAL_GROUND;
  // }

  public IntakeState getState() {
    return state;
  }

  public IntakeSubsystem(
      RollerIO rollerIO,
      PivotIO pivotIO,
      CANrangeIO leftCanrangeIO,
      CANrangeIO rightCanrangeIO,
      String name) {
    super(rollerIO, pivotIO, name);
    this.frontCanrangeIO = leftCanrangeIO;
    this.rearCanrangeIO = rightCanrangeIO;
  }

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public void setState(IntakeState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    super.periodic();
    frontCanrangeIO.updateInputs(leftCanrangeInputs);
    Logger.processInputs("Intake/frontCanrange", leftCanrangeInputs);
    rearCanrangeIO.updateInputs(rightCanrangeInputs);
    Logger.processInputs("Intake/rearCanrange", rightCanrangeInputs);
  }

  public double getfrontCanrangePosition() {
    return leftCanrangeInputs.distanceCm;
  }

  public double getRearCanrangePosition() {
    return rightCanrangeInputs.distanceCm;
  }

  @AutoLogOutput(key = "Intake/HasGamePiece")
  public boolean hasGamePiece() {
    return getfrontCanrangePosition() < 10.0 || getRearCanrangePosition() < 10.0;
  }

  public Command zeroPivot() {
    return Commands.runOnce(
        () -> {
          intakeHasZeroed = true;
          System.out.println("Intake zeroed!");
          currentPosition = Rotation2d.fromDegrees(ZEROING_OFFSET);
        });
  }

  public Command zeroIntake() {
    return Commands.run(() -> setPivotVoltage(-2))
        .until(() -> Math.abs(currentFilterValue) > 10.0)
        .andThen(zeroPivot());
  }

  public void setIntakePosition(Rotation2d target) {
    if (intakeHasZeroed) {
      currentPosition = target;
    }
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public Command setStateAngleVoltage(IntakeState state) {
    return this.runOnce(
        () -> {
          setPivotAngle(state.position);
          runRollerVoltage(state.volts);
        });
  }
}
