package frc.robot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Pivot.PivotIO;
import frc.robot.Pivot.PivotIOInputsAutoLogged;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;
import frc.robot.rollerAndPivot.rollerPivotSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends rollerPivotSubsystem {
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final String name;
  private final RollerIO rollers;
  private final PivotIO pivot;

  public ClimberSubsystem(RollerIO rollers, PivotIO pivot, String name) {
    super(rollers, pivot, name);
    this.name = name;
    this.rollers = rollers;
    this.pivot = pivot;
  }

  @AutoLogOutput(key = "Climber/State")
  private ClimberState state = ClimberState.IDLE;

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

  @Override
  public void periodic() {
    pivot.updateInputs(pivotInputs);
    Logger.processInputs(name + "/Pivot", pivotInputs);
    rollers.updateInputs(rollerInputs);
    Logger.processInputs(name + "/Roller", rollerInputs);
  }

  public Command runRollerVoltage(double volts) {
    return this.run(() -> rollers.setRollerVoltage(volts));
  }

  public Command setTargetAngle(Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Command setTargetAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Arm", target.get());
          pivot.setMotorPosition(target.get());
        });
  }
}
