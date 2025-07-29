package frc.robot.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmSubsystem extends SubsystemBase {
    //constants
    //TODO: change to real values
    public static final double PIVOT_RATIO = 10; 
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    public Command setTargetAngle(Rotation2d target) {
        return setTargetAngle(() -> target);
    }

    public Command setTargetAngle(Supplier<Rotation2d> target) {
        return this.runOnce(
           () -> {
             if (Robot.ROBOT_TYPE != RobotType.REAL)
               Logger.recordOutput("arm", target.get());
           })
       .andThen(this.run(() -> io.setMotorPosition(target.get())));
    }

    public void setMotorVoltage(double voltage) {
        io.setMotorVoltage(voltage);
    }

    public void setMotorPosition(Rotation2d targetPosition) {
        io.setMotorPosition(targetPosition);
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    //TODO : change these values to the real ones 
    public enum ArmState {
        IDLE(Rotation2d.fromDegrees(0)),
        //coral
        READY_CORAL(Rotation2d.fromDegrees(0)),
        PRE_INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(0)),
        INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(0)),
        L1(Rotation2d.fromDegrees(0)),
        PRE_L2(Rotation2d.fromDegrees(0)),
        L2(Rotation2d.fromDegrees(0)),
        PRE_L3(Rotation2d.fromDegrees(0)),
        L3(Rotation2d.fromDegrees(0)),
        PRE_L4(Rotation2d.fromDegrees(0)),
        L4(Rotation2d.fromDegrees(0)),
        POST_L4(Rotation2d.fromDegrees(0)),
        //algae
        INTAKE_ALGAE_REEF_HIGH(Rotation2d.fromDegrees(0)),
        INTAKE_ALGAE_REEF_LOW(Rotation2d.fromDegrees(0)),
        INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(0)),
        BARGE(Rotation2d.fromDegrees(0)),
        READY_ALGAE(Rotation2d.fromDegrees(0)),
        PROCESSOR(Rotation2d.fromDegrees(0)),
        //climbing
        PRE_CLIMB(Rotation2d.fromDegrees(0)),
        CLIMB(Rotation2d.fromDegrees(0));

        public final Rotation2d position;

        ArmState(Rotation2d position) {
            this.position = position;
        }
    }
}
