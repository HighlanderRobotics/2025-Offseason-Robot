package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  public static final double PIVOT_RATIO = 12.5; // (15.0 / 1);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double LENGTH_METERS = 0.325;
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  // TODO tune
  // TODO THESE SUCK !
  public static final double KP = 80.0;
  public static final double KI = 5.0;
  public static final double KD = 3.0;
  public static final double KS = 0.381;
  public static final double KG = 2.0;
  public static final double KV = 0.1;
  public static final double jKgMetersSquared = 0.01;
  public static final double TOLERANCE_DEGREES = 10.0;
  private final CANrangeIO leftCanrangeIO;
  private final CANrangeIO rightCanrangeIO;
  private final CANrangeIOInputsAutoLogged leftCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final CANrangeIOInputsAutoLogged rightCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final Rotation2d ZEROING_POSITION = Rotation2d.fromDegrees(-10.0);
  private final double CURRENT_THRESHOLD = 10.0;

  private boolean hasGamePieceSim = false;

  public boolean intakeZeroed = false;

  // TODO : change these values to the real ones
  public enum IntakeState {
    IDLE(0, 0.0),
    INTAKE_CORAL(Units.radiansToDegrees(-2.05), 15.0),
    READY_CORAL_INTAKE(0.0, 1.0),
    HANDOFF(Units.radiansToDegrees(1.96), -15.0),
    PRE_L1(-90, 1.0),
    SCORE_L1(-90, -5.0),
    CLIMB(0, 0.0);

    public final Rotation2d position;
    public final double velocityRPS;

    private IntakeState(double positionDegrees, double velocityRPS) {
      this.position = Rotation2d.fromDegrees(positionDegrees);
      this.velocityRPS = velocityRPS;
    }

    public Rotation2d getAngle() {
      return position;
    }

    public double getVelocityRPS() {
      return velocityRPS;
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

  public Command rezero() {
    return this.run(() -> pivotIO.resetEncoder(Rotation2d.kZero));
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public Command setStateAngleVoltage() {
    return this.run(
        () -> {
          Logger.recordOutput("Intake/Pivot Setpoint", state.position);
          pivotIO.setMotorPosition(state.position, hasGamePiece() ? 1 : 0);
          rollerIO.setRollerVelocity(state.velocityRPS);
        });
  }

  public static TalonFXConfiguration getIntakePivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Slot 0 is for without a coral
    config.Slot0.kV = 1;
    config.Slot0.kG = 1.05;
    config.Slot0.kS = 0.38;
    config.Slot0.kP = 15;
    config.Slot0.kI = 0.1;
    config.Slot0.kD = 1;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Slot 1 is with a coral
    config.Slot1.kP = 50;
    config.Slot1.kI = 0;
    config.Slot1.kD = 5;
    config.Slot1.kS = 0.32;
    config.Slot1.kV = 3;
    config.Slot1.kG = 1.36;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 12.5;

    return config;
  }
}
