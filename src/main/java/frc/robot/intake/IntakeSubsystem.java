package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.canrange.CANrangeIO;
import frc.robot.canrange.CANrangeIOInputsAutoLogged;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = (15.0 / 1);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double LENGTH_METERS = 0.325;
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
  public static final double TOLERANCE_DEGREES = 10.0;
  private final CANrangeIO leftCanrangeIO;
  private final CANrangeIO rightCanrangeIO;
  private final CANrangeIOInputsAutoLogged leftCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final CANrangeIOInputsAutoLogged rightCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final Rotation2d ZEROING_POSITION = Rotation2d.fromDegrees(-10.0);
  private final double CURRENT_THRESHOLD = 10.0;

  public boolean intakeZeroed = false;

  // TODO : change these values to the real ones
  public enum IntakeState {
    IDLE(0, 0.0),
    INTAKE_CORAL(0, 10.0),
    READY_CORAL_INTAKE(-130, 1.0),
    HANDOFF(-130, -5.0),
    PRE_L1(-90, 1.0),
    SCORE_L1(-90, -5.0),
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
    this.leftCanrangeIO = leftCanrangeIO;
    this.rightCanrangeIO = rightCanrangeIO;
  }

  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.IDLE;

  public void setState(IntakeState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    super.periodic();
    leftCanrangeIO.updateInputs(leftCanrangeInputs);
    Logger.processInputs("Intake/Left CANrange", leftCanrangeInputs);
    rightCanrangeIO.updateInputs(rightCanrangeInputs);
    Logger.processInputs("Intake/Right CANrange", rightCanrangeInputs);
  }

  public double getleftCanrangeDistanceMeters() {
    return leftCanrangeInputs.distanceMeters;
  }

  public double getRightCanrangeDistanceMeters() {
    return rightCanrangeInputs.distanceMeters;
  }

  @AutoLogOutput(key = "Intake/Has Game Piece")
  public boolean hasGamePiece() {
    return getleftCanrangeDistanceMeters() < 0.05 || getRightCanrangeDistanceMeters() < 0.05;
  }

  public Command zeroIntake() {
    return this.run(() -> setPivotAngle(() -> Rotation2d.fromDegrees(-80)))
        .until(new Trigger(() -> Math.abs(currentFilterValue) > CURRENT_THRESHOLD).debounce(0.25))
        .andThen(
            Commands.parallel(
                Commands.runOnce(() -> intakeZeroed = true),
                Commands.print("Intake Zeroed"),
                zeroPivot(() -> ZEROING_POSITION)));
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public Command setStateAngleVoltage() {
    return setPivotAndRollers(() -> state.position, () -> state.volts);
  }
}
