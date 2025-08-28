package frc.robot.Pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOSim implements PivotIO {
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();

  public PivotIOSim() {}

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angularVelocityRotsPerSec = 0.0;
    inputs.position = new Rotation2d(0.0);
    inputs.cancoderPosition = new Rotation2d(0.0);
    inputs.supplyCurrentAmps = 0.0;
    inputs.appliedVoltage = 0.0;
    inputs.statorCurrentAmps = 0.0;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    // Simulate setting motor voltage
  }

  @Override
  public void setMotorPosition(Rotation2d targetPosition) {
    // Simulate setting motor position
  }
}
