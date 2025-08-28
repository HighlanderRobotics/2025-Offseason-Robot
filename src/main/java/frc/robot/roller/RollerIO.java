package frc.robot.roller;

import org.littletonrobotics.junction.AutoLog;

// import edu.wpi.first.math.geometry.Rotation2d;

public interface RollerIO {

  @AutoLog
  public class RollerIOInputs {
    public double angularVelocityRotsPerSec = 0.0;
    // public Rotation2d position = new Rotation2d();
    public double supplyCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public void updateInputs(RollerIOInputs inputs);

  public void setRollerVoltage(double volts);
}
