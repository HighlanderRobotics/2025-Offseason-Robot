// package frc.robot.Pivot;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.Robot.RobotType;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;

// public class PivotSubsystem extends SubsystemBase {

//   private PivotIO io;
//   private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
//   private final String name;

//   public boolean CANcoder = false;

//   public void setCANcoder() {
//     this.CANcoder = true;
//   }

//   public PivotSubsystem(PivotIO io, String name) {
//     this.io = io;
//     this.name = name;
//   }

//   @Override
//   public void periodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs(name, inputs);
//   }

//   public Command setTargetAngle(Rotation2d target) {
//     return setTargetAngle(() -> target);
//   }

//   public Command setTargetAngle(Supplier<Rotation2d> target) {
//     return this.runOnce(
//         () -> {
//           if (Robot.ROBOT_TYPE != RobotType.REAL) Logger.recordOutput("Arm", target.get());
//           io.setMotorPosition(target.get());
//         });
//   }
// }
