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

  // TODO : change these values to the real ones
  public enum ArmState {
    // TODO check positive direction
    // 0 for position is horizontal pointed right. positive is counterclockwise
    // positive voltage is intaking, negative is outtaking
    IDLE(Rotation2d.fromDegrees(0), 0.0),
    // coral
    HANDOFF(Rotation2d.fromDegrees(0), 5.0),
    READY_CORAL(Rotation2d.fromDegrees(0), 1.0),
    INTAKE_CORAL_STACK(Rotation2d.fromDegrees(190), 5.0),

    PRE_L2(Rotation2d.fromDegrees(45), 0.0),
    SCORE_L2(Rotation2d.fromDegrees(0), -10.0),
    PRE_L3(Rotation2d.fromDegrees(45), 0.0),
    SCORE_L3(Rotation2d.fromDegrees(0), -10.0),
    PRE_L4(Rotation2d.fromDegrees(20), 0.0),
    SCORE_L4(Rotation2d.fromDegrees(0), -10.0),
    // algae
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(0), 10.0),
    INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(215), 15.0),
    INTAKE_ALGAE_STACK(Rotation2d.fromDegrees(180), 10.0),
    READY_ALGAE(Rotation2d.fromDegrees(90), 2.0),

    PRE_BARGE(Rotation2d.fromDegrees(110), 4.0),
    SCORE_BARGE(Rotation2d.fromDegrees(110), -10.0),
    PRE_PROCESSOR(Rotation2d.fromDegrees(180), 0.0),
    SCORE_PROCESSOR(Rotation2d.fromDegrees(180), -10.0),
    // climbing
    PRE_CLIMB(Rotation2d.fromDegrees(180), 0.0),
    CLIMB(Rotation2d.fromDegrees(180), 0.0);

    public final Rotation2d position;
    public final double volts;

    private ArmState(Rotation2d position, double volts) {
      this.position = position;
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
