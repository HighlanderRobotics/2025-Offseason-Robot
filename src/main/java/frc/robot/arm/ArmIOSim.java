// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ArmSubsystem.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(
              Units.inchesToMeters(25.212),
              Units.lbsToKilograms(6.518)), // ngl idk where to start measuring the length from
          Units.inchesToMeters(25.212),
          -Math.PI,
          Math.PI,
          true,
          0);

  private final ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0); // 1.31085, 0.278);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          80.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0)); // magic!

  private double volts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
        if (DriverStation.isDisabled()) sim.setInput(0);
    sim.update(0.02);
    inputs.motorPosition = Rotation2d.fromRadians(sim.getAngleRads());
    Logger.recordOutput("Arm/Sim position", sim.getAngleRads());
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

  // i'm going to Ignore this!
  @Override
  public void setEncoderPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
  }
}
