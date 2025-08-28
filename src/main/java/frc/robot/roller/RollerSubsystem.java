// package frc.robot.roller;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;

// public class RollerSubsystem extends SubsystemBase {
//   public RollerIO io;
//   private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
//   public final String name;

//   public RollerSubsystem(RollerIO io, String name) {
//     this.io = io;
//     this.name = name;
//   }

//   @Override
//   public void periodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs(name, inputs);
//   }

//   public Command setRollerVoltage(double volts) {
//     return setRollerVoltage(volts);
//   }

//   public Command runRollerVoltage(double volts) {
//     return this.run(() -> io.setRollerVoltage(volts));
//   }
// }
