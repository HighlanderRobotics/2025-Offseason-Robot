package frc.robot.routing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class RoutingSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final String name;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RoutingSubsystem(RollerIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public Command setRollerVoltage(double volts) {
    return setRollerVoltage(volts);
  }

  public Command runRollerVoltage(double volts) {
    return this.run(() -> io.setRollerVoltage(volts));
  }
}
