// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOCTRESim extends ElevatorIOReal {
  TalonFXSimState leaderTalonSim;
  TalonFXSimState followerTalonSim;

  private final ElevatorSim elevatorPhysicsSim =
      new ElevatorSim(
          //   DCMotor.getKrakenX60Foc(2),
          // for 2 kraken x44s
          new DCMotor(12, 5.01, 329, 2, Units.rotationsPerMinuteToRadiansPerSecond(7368), 2),
          ElevatorSubsystem.GEAR_RATIO,
          // Add half of first stage mass bc its on a 2:1 ratio compared to carriage
          // First stage weighs 3.345 lbs
          // Carriage weighs 8.863
          // Arm weighs 5.625
          Units.lbsToKilograms((3.345 / 2) + 8.863 + 5.625),
          ElevatorSubsystem.SPROCKET_DIAMETER_METERS / 2,
          0.0,
          ElevatorSubsystem.MAX_EXTENSION_METERS,
          true,
          0.0);

  // double m_lastSimTime;
  // Notifier m_simNotifier;

  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier simNotifier = null;
  private double lastSimTime = 0.0;

  public ElevatorIOCTRESim() {
    super();
    leaderTalonSim = leader.getSimState();
    leaderTalonSim.setMotorType(MotorType.KrakenX44);
    leaderTalonSim.Orientation = ChassisReference.Clockwise_Positive;
    followerTalonSim = follower.getSimState();
    followerTalonSim.setMotorType(MotorType.KrakenX44);
    followerTalonSim.Orientation = ChassisReference.Clockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              /* Calculate the time delta */
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* First set the supply voltage of all the devices */
              leaderTalonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              /* Then calculate the new position and velocity of the simulated elevator */
              elevatorPhysicsSim.setInputVoltage(leaderTalonSim.getMotorVoltage());
              elevatorPhysicsSim.update(deltaTime);

              /* Apply the new rotor position and velocity to the motors (before gear ratio) */
              leaderTalonSim.setRawRotorPosition(
                  elevatorPhysicsSim.getPositionMeters()
                      / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
                      * ElevatorSubsystem.GEAR_RATIO);
              // convert meters/second -> rotations/second
              leaderTalonSim.setRotorVelocity(
                  elevatorPhysicsSim.getVelocityMetersPerSecond()
                      / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
                      * ElevatorSubsystem.GEAR_RATIO);
            });
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    // if (DriverStation.isDisabled()) {
    //   stop();
    // }
    // // set the supply voltage of the motor
    // leaderTalonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // elevatorPhysicsSim.setInputVoltage(leaderTalonSim.getMotorVoltage());
    // elevatorPhysicsSim.update(0.005); // assume 5 ms loop time because onboard pid

    // // update the ctre motor sim to match the wpilib physics sim
    // // note that "set" does not command the motor to do anything
    // // it just updates the value the ctre sim has stored

    // // convert meters -> rotations for rotor position because it doesn't let you set linear
    // position
    // // directly
    // leaderTalonSim.setRawRotorPosition(
    //     elevatorPhysicsSim.getPositionMeters()
    //         / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
    //         * ElevatorSubsystem.GEAR_RATIO);
    // // convert meters/second -> rotations/second
    // leaderTalonSim.setRotorVelocity(
    //     elevatorPhysicsSim.getVelocityMetersPerSecond()
    //         / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
    //         * ElevatorSubsystem.GEAR_RATIO);
    // Logger.recordOutput("sim vel", elevatorPhysicsSim.getVelocityMetersPerSecond());
    // Logger.recordOutput(
    //     "hopefully rotor vel",
    //     elevatorPhysicsSim.getVelocityMetersPerSecond()
    //         * (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
    //         / ElevatorSubsystem.GEAR_RATIO);

    // followerTalonSim.setRawRotorPosition(
    //     elevatorPhysicsSim.getPositionMeters()
    //         / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
    //         * ElevatorSubsystem.GEAR_RATIO);
    // // setting the follower velocity does not seem to do anything
    // followerTalonSim.setRotorVelocity(
    //     elevatorPhysicsSim.getVelocityMetersPerSecond()
    //         / (Math.PI * ElevatorSubsystem.SPROCKET_DIAMETER_METERS)
    //         * ElevatorSubsystem.GEAR_RATIO);

    // updates the values in the logtable
    // it will pull those values from the motor (same as irl)
    super.updateInputs(inputs);
  }
}
