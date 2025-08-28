package frc.robot.roller;

public class RollerIOSim implements RollerIO {
  // deal with sim later uses like a motor sim thing
  private final RollerIO.RollerIOInputs inputs = new RollerIO.RollerIOInputs();

  public RollerIOSim() {}

  public void updateInputs(RollerIOInputs inputs) {
    inputs.angularVelocityRotsPerSec = 0.0;
    inputs.supplyCurrentAmps = 0.0;
    inputs.appliedVoltage = 0.0;
    inputs.statorCurrentAmps = 0.0;
  }

  public void setRollerVoltage(double volts) {
    // later
  }
}
