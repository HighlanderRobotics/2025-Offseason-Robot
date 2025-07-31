// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IntakeSubsystem.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(
              Units.inchesToMeters(18.36),
              Units.lbsToKilograms(12)), // TODO not all materials are defined yet
          Units.inchesToMeters(18.36),
          0,
          Math.PI,
          true,
          0);

  private final ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0); // 1.31085, 0.278);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          8.0, 0.0, 0.0, new TrapezoidProfile.Constraints(50.0, 50.0)); // magic!

  private double volts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) sim.setInput(0);
    sim.update(0.02);
    inputs.motorPosition = Rotation2d.fromRadians(sim.getAngleRads());
    Logger.recordOutput("Intake/Sim position", sim.getAngleRads());
    // can't sim cancoder i guess
    inputs.pivotVoltage = volts;
  }

  @Override
  public void setPivotVoltage(double voltage) {
    volts = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setPivotAngle(Rotation2d position) {
    setPivotVoltage(
        pid.calculate(sim.getAngleRads(), position.getRadians())
            + ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
  }
}
