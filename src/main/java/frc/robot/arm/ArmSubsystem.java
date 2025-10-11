package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  // constants
  // TODO: change to real values
  public static final double PIVOT_RATIO = (44.0 / 16.0) * 23;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  @AutoLogOutput(key = "Arm/State")
  private ArmState state = ArmState.IDLE;

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command setTargetAngle(Rotation2d target) {
    return setTargetAngle(() -> target);
  }

  public Command setTargetAngle(Supplier<Rotation2d> target) {
    return this.runOnce(
        () -> {
          if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Arm", target.get());
          io.setMotorPosition(target.get());
        });
  }

  public void setState(ArmState state) {
    this.state = state;
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.position.getDegrees(), 10.0);
  }

  // TODO setStateAngleVoltage
  public Command setStateAngleVoltage() {
    return Commands.none();
  }

  // TODO hasCoral
  // current spike
  public boolean hasCoral() {
    return true;
  }

  // TODO hasAlgae
  // current spike
  // Unclear if this will be distinct from the coral current spike?
  public boolean hasAlgae() {
    return true;
  }

  // TODO setSimCoral
  public void setSimCoral(boolean b) {}
}
