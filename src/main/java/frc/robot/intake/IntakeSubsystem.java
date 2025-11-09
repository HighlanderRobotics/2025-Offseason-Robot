package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
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
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = 12.5; // (15.0 / 1);
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(130);
  public static final Rotation2d MIN_ANGLE =
      Rotation2d.fromRadians(-0.3); // Rotation2d.fromDegrees(0);
  public static final Rotation2d ZEROING_ANGLE =
      Rotation2d.fromRadians(-0.5); // (-0.42184471666855133);
  public static final double LENGTH_METERS = 0.325;
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  // for mech viz
  public static final double VERTICAL_OFFSET_METERS = Units.inchesToMeters(7.566);

  public static final double KP = 47; // 80.0;
  public static final double KI = 0.0; // 5.0;
  public static final double KD = 4.7; // 3.0;
  public static final double KS = 0.3203125; // 0.381;
  public static final double KG = 0.3603515625; // 2.0;
  public static final double KV = 2; // 0.1;
  public static final double jKgMetersSquared = 0.01;
  public static final double TOLERANCE_DEGREES = 10.0;
  private final CANrangeIO leftCanrangeIO;
  private final CANrangeIO rightCanrangeIO;
  private final CANrangeIOInputsAutoLogged leftCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final CANrangeIOInputsAutoLogged rightCanrangeInputs = new CANrangeIOInputsAutoLogged();
  private final Rotation2d ZEROING_POSITION = Rotation2d.fromRadians(-0.5);
  public static final double CURRENT_THRESHOLD = 60.0;

  private LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(10);
  private double pivotCurrentFilterValue = 0.0;

  public enum IntakeState {
    IDLE(Units.radiansToDegrees(1.96), 0.0),
    INTAKE_CORAL(Units.radiansToDegrees(-0.5), 17.0),
    READY_CORAL_INTAKE(Units.radiansToDegrees(1.96), 1.0),
    HANDOFF(110.867, -17.0), // Units.radiansToDegrees(1.96)
    PRE_L1(76, 1.0),
    SCORE_L1(76, -7.0),
    CLIMB(Units.radiansToDegrees(-0.5), 0.0);

    public final Supplier<Rotation2d> position;
    public final DoubleSupplier velocityRPS;

    private IntakeState(double positionDegrees, double velocityRPS) {
      LoggedTunableNumber ltn =
          new LoggedTunableNumber("Intake/Angle: " + this.name(), positionDegrees);
      // we're in real life!! use degrees
      this.position = () -> Rotation2d.fromDegrees(ltn.get());
      this.velocityRPS = new LoggedTunableNumber("Intake/Velocity: " + this.name(), velocityRPS);
    }

    public Rotation2d getAngle() {
      return position.get();
    }

    public double getVelocityRPS() {
      return velocityRPS.getAsDouble();
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

  private boolean hasGamePieceSim = false;

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

    pivotCurrentFilterValue = pivotCurrentFilter.calculate(pivotInputs.statorCurrentAmps);
  }

  public double getleftCanrangeDistanceMeters() {
    return leftCanrangeInputs.distanceMeters;
  }

  public double getRightCanrangeDistanceMeters() {
    return rightCanrangeInputs.distanceMeters;
  }

  @AutoLogOutput(key = "Intake/Has Game Piece")
  public boolean hasGamePiece() {
    // return getleftCanrangeDistanceMeters() < 0.01 || getRightCanrangeDistanceMeters() < 0.01;
    return leftCanrangeInputs.isDetected || rightCanrangeInputs.isDetected;
  }

  public Command runCurrentZeroing() {
    return setPivotVoltage(() -> -5.0)
        .until(
            new Trigger(() -> Math.abs(pivotCurrentFilterValue) > CURRENT_THRESHOLD).debounce(0.25))
        .andThen(
            Commands.parallel(Commands.print("Intake Zeroed"), zeroPivot(() -> ZEROING_POSITION)));
  }

  public Command rezero() {
    // return this.runOnce(() -> pivotIO.resetEncoder(Rotation2d.kCCW_90deg));
    return this.runOnce(() -> pivotIO.resetEncoder(ZEROING_ANGLE));
  }

  public Command ninety() {
    // return this.runOnce(() -> pivotIO.resetEncoder(Rotation2d.kCCW_90deg));
    return this.runOnce(() -> pivotIO.resetEncoder(Rotation2d.fromDegrees(90)));
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public Command setStateAngleVelocity() {
    return this.run(
        () -> {
          // int slot = hasGamePiece() ? 1 : 0;
          Logger.recordOutput("Intake/Pivot Setpoint", state.position.get());
          // pivotIO.setMotorPosition(state.position.get(), slot);

          pivotIO.setMotorPosition(state.position.get(), 1);
          rollerIO.setRollerVelocity(state.velocityRPS.getAsDouble());
        });
    // return this.run(() -> pivotIO.setMotorPosition(SuperState.IDLE.intakeState.getAngle()));

    // this is wrong?
    // return this.run(() -> setPivotAndRollers(getState().position, getState().velocityRPS));
  }

  public double getPivotCurrentFilterValueAmps() {
    return pivotCurrentFilterValue;
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
    config.Slot1.kP = 60;
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

  public Command setRollerVelocity(DoubleSupplier vel) {
    return this.run(() -> runRollerVelocity(vel.getAsDouble()));
  }
}
