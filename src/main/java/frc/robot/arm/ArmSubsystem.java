package frc.robot.arm;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.Superstructure.SuperState;
import frc.robot.cancoder.CANcoderIO;
import frc.robot.cancoder.CANcoderIOInputsAutoLogged;
import frc.robot.pivot.PivotIO;
import frc.robot.roller.RollerIO;
import frc.robot.rollerpivot.RollerPivotSubsystem;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends RollerPivotSubsystem {
  public static final double PIVOT_RATIO = 79.17;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  public static final double LENGTH_METERS = 0.659;
  public static final double MAX_ACCELERATION = 10.0;
  public static final double MAX_VELOCITY = 10.0;
  public static final double ROLLERS_RATIO = (44.0 / 16.0) * 23;
  public static final double ZEROING_CURRENT_THRESHOLD_AMPS = 30.0;

  public static final double ZEROING_ANGLE = -108;

  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SENSOR_TO_MECH_RATIO = PIVOT_RATIO;
  public static final double KP = 120.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KG = 0.4;
  public static final double KV = 0.14;
  public static final double jKgMetersSquared = 0.01;
  // public static final double GAME_PIECE_CURRENT_THRESHOLD = 20.0;
  // public static final double ALGAE_INTAKE_VOLTAGE = 8.0;
  public static final double CORAL_INTAKE_VOLTAGE = 5.0;
  // public static final double ALGAE_CURRENT_THRESHOLD = 20.0;
  public static final double CORAL_CURRENT_THRESHOLD = 30.0;
  public static final double TOLERANCE_DEGREES = 10.0;
  public static final double VERTICAL_OFFSET_METERS = Units.inchesToMeters(12.0);
  public static final double SAFE_ZEROING_ANGLE = -80.0; // idk

  public static final double CANCODER_OFFSET = -0.3545; // -0.368896484375;
  // this is because we want it to wrap around from -180 to 180, which is when it's pointed straight
  // down
  public static final double CANCODER_DISCONTINUITY_POINT = 0.5;

  private final CANcoderIO cancoderIO;
  private final CANcoderIOInputsAutoLogged cancoderInputs = new CANcoderIOInputsAutoLogged();

  // public boolean hasAlgae = false;
  public boolean hasCoral = false;

  public boolean armZeroed = false;

  private LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(10);
  private double pivotCurrentFilterValue = 0.0;

  /**
   * 0 for position is vertical with the EE up. We consider the front of the robot to be the intake,
   * so left and right are based on that. Positive is counterclockwise from the robot's POV (which
   * is to the left). We're in real life!! use degrees. degrees -> Rotation2d gets handled in the
   * constructor Positive voltage is intaking, negative is outtaking.
   */
  public enum ArmState {
    IDLE(0, 0.0),
    // coral
    PRE_RIGHT_HANDOFF(90, 7.0),
    RIGHT_HANDOFF(180, 7.0),
    RIGHT_POST_HANDOFF(89, 7.0),

    PRE_LEFT_HANDOFF(-90, 7.0),
    LEFT_HANDOFF(-180, 7.0),
    LEFT_POST_HANDOFF(-89, 7.0),

    INTAKE_CORAL_STACK(-100, 7.0),
    READY_CORAL_ARM(0, 7.0),

    PRE_L2_RIGHT(-35, 7.0),
    SCORE_L2_RIGHT(-90, -10.0),
    PRE_L3_RIGHT(-45, 7.0),
    SCORE_L3_RIGHT(-90, -10.0),
    PRE_L4_RIGHT(-60, 10.0),
    SCORE_L4_RIGHT(-90, -14.0),

    PRE_L2_LEFT(45, 7.0),
    SCORE_L2_LEFT(90, -10.0),
    PRE_L3_LEFT(45, 7.0),
    SCORE_L3_LEFT(90, -10.0),
    PRE_L4_LEFT(70, 10.0),
    SCORE_L4_LEFT(90, -10.0),
    // algae
    // These all have voltage control
    // INTAKE_ALGAE_REEF_RIGHT(-90, 10.0),
    // INTAKE_ALGAE_REEF_LEFT(90, 10.0),
    // INTAKE_ALGAE_GROUND(125, 15.0),
    // // TODO: SET POS BACK TO 90 deg
    // INTAKE_ALGAE_STACK(90, 8.0),
    // READY_ALGAE(0, 8.0),

    // PRE_BARGE_RIGHT(-20, 4.0),
    // SCORE_BARGE_RIGHT(-20, -10.0),

    // PRE_BARGE_LEFT(20, 4.0),
    // SCORE_BARGE_LEFT(20, -10.0),

    // PRE_PROCESSOR(180, 0.0),
    // SCORE_PROCESSOR(180, -10.0),
    // climbing
    PRE_CLIMB(-108, 0.0),
    CLIMB(-108, 0.0);

    public final Supplier<Rotation2d> position;
    public final DoubleSupplier velocityRPS;

    private ArmState(double positionDegrees, double velocityRPS) {
      LoggedTunableNumber ltn =
          new LoggedTunableNumber("Arm/Angle: " + this.name(), positionDegrees);
      // we're in real life!! use degrees
      this.position = () -> Rotation2d.fromDegrees(ltn.get());
      this.velocityRPS = new LoggedTunableNumber("Arm/Velocity: " + this.name(), velocityRPS);
    }

    public Rotation2d getAngle() {
      return position.get();
    }

    public double getVelocityRPS() {
      return velocityRPS.getAsDouble();
    }
  }

  public ArmSubsystem(RollerIO rollerIO, PivotIO pivotIO, CANcoderIO cancoderIO, String name) {
    super(rollerIO, pivotIO, name);
    this.cancoderIO = cancoderIO;

    new Trigger(() -> Math.abs(currentFilterValue) > CORAL_CURRENT_THRESHOLD)
        .debounce(0.25)
        .whileTrue(Commands.runOnce(() -> hasCoral = true))
        .whileFalse(Commands.runOnce(() -> hasCoral = false));
    ;
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

    pivotCurrentFilterValue = pivotCurrentFilter.calculate(pivotInputs.statorCurrentAmps);
  }

  public Rotation2d getCANcoderPosition() {
    return cancoderInputs.cancoderPositionRotations;
  }

  // public Command intakeAlgae() {
  //   return this.run(() -> runRollerVoltage(() -> ALGAE_INTAKE_VOLTAGE))
  //       .until(
  //           new Trigger(() -> Math.abs(currentFilterValue) > ALGAE_CURRENT_THRESHOLD)
  //               .debounce(0.25))
  //       .andThen(Commands.runOnce(() -> hasAlgae = true));
  // }

  public Command intakeCoral() {
    return this.run(() -> runRollerVoltage(CORAL_INTAKE_VOLTAGE))
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
    // return hasAlgae || hasCoral;
    return hasCoral;
  }

  public boolean isNearAngle(Rotation2d target) {
    return isNear(target, TOLERANCE_DEGREES);
  }

  public Command setStateAngleVelocity() {
    // return setPivotAndRollers(
    //     () -> getState().position.get(), () -> getState().velocityRPS.getAsDouble());
    // return this.run(() -> runRollerVelocity(getState().velocityRPS.getAsDouble()));
    return this.run(
        () -> {
          if (Superstructure.getState() == SuperState.CLIMB
              || Superstructure.getState() == SuperState.PRE_CLIMB) {
            pivotIO.setMotorVoltage(-3.0);
          } else {
            pivotIO.setMotorPosition(state.getAngle());
          }
          // // TODO: THIS SUCKS
          // if (Superstructure.stateIsVoltageControl()) {
          //   rollerIO.setRollerVoltage(state.getVelocityRPS());
          // } else {
          //   rollerIO.setRollerVelocity(state.getVelocityRPS());
          // }

          rollerIO.setRollerVelocity(state.getVelocityRPS());
        });
  }

  public Command hold() {
    return Commands.sequence(
        Commands.runOnce(() -> setPivotAngle(pivotInputs.position))
            // holds
            .until(() -> true),
        // keeps command active until interrupted
        this.run(() -> {}));
  }

  public void setSimCoral(boolean hasCoral) {
    if (Robot.isSimulation()) {
      this.hasCoral = hasCoral;
    }
  }

  // public void setSimAlgae(boolean hasAlgae) {
  //   if (Robot.isSimulation()) {
  //     this.hasAlgae = hasAlgae;
  //   }
  // }

  public Command rezeroFromEncoder() {
    return zeroPivot(() -> getCANcoderPosition());
  }

  public Command rezeroAgainstRightBumper() {
    return zeroPivot(() -> Rotation2d.fromDegrees(ZEROING_ANGLE));
  }

  public void setHasCoralForAuto(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public Command runCurrentZeroing() {
    return setPivotVoltage(() -> -3.0)
        .until(
            new Trigger(() -> Math.abs(pivotCurrentFilterValue) > ZEROING_CURRENT_THRESHOLD_AMPS)
                .debounce(0.25))
        .andThen(
            Commands.parallel(
                Commands.runOnce(() -> armZeroed = true),
                Commands.print("Arm Zeroed"),
                rezeroAgainstRightBumper()));
  }

  public Command setPivotAngle(Supplier<Rotation2d> rot) {
    return this.run(() -> setPivotAngle(rot.get()));
  }

  public Command setRollerVelocity(DoubleSupplier vel) {
    return this.run(() -> runRollerVelocity(vel.getAsDouble()));
  }

  public double getPivotCurrentFilterValueAmps() {
    return pivotCurrentFilterValue;
  }
}
