package frc.robot.routing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.beambreak.BeamBreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.roller.RollerIO;
import frc.robot.roller.RollerIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RoutingSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final String name;
  private final BeamBreakIO beamBreak;

  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private BeambreakIOInputsAutoLogged beamBreakInputs = new BeambreakIOInputsAutoLogged();

  public RoutingSubsystem(RollerIO io, BeamBreakIO beamBreak, String name) {
    this.io = io;
    this.beamBreak = beamBreak;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name + "/Roller", inputs);
    beamBreak.updateInputs(beamBreakInputs);
    Logger.processInputs(name + "/BeamBreak", beamBreakInputs);
  }

  public Command setRollerVoltage(double volts) {
    return setRollerVoltage(volts);
  }

  public Command runRollerVoltage(DoubleSupplier volts) {
    return this.run(() -> io.setRollerVoltage(volts.getAsDouble()));
  }

  public Command index() {
    return Commands.sequence(
        this.run(() -> io.setRollerVelocity(9.0)).until(() -> beamBreakInputs.get),
        this.run(() -> io.setRollerVelocity(0)));
  }
}
