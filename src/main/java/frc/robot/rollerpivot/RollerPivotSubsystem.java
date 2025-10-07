package frc.robot.rollerpivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.pivot.PivotIO;
import frc.robot.pivot.PivotIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class RollerPivotSubsystem extends SubsystemBase {
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final RollerIO rollerIO;
  private final PivotIO pivotIO;
  private final String name;

  public RollerPivotSubsystem(RollerIO rollerIO, PivotIO pivotIO, String name) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.name = name;
  }

  public Command runRollerVoltage(double volts) {
    return this.run(() -> rollerIO.setRollerVoltage(volts));
  }

  public Command setPivotAngle(Rotation2d target) {
    return this.runOnce(
        () -> {
          if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("/Pivot Setpoint", target);
          pivotIO.setMotorPosition(target);
        });
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs(name + "/Pivot", pivotInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(name + "/Roller", rollerInputs);
  }
}
