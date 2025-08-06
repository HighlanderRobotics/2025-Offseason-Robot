// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  //TODO put in proper swerve constants later
  private double MAX_LINEAR_SPEED = Units.feetToMeters(17.1);
  private final Module[] modules; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(ModuleIO[] moduleIOs) {
    modules = new Module[moduleIOs.length];
    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }
  }

  @Override
  public void periodic() {
    for (Module m : modules) {
      m.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
  }

  private void driveOpenLoop(ChassisSpeeds speeds) {
    //makes it actually usable in the 20ms increments
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    //breaks those speeds up into individual states to send to the modules
    final SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    //if it's faster than the module can physically go it will reduce that
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    for (int i = 0; i < setpointStates.length; i++) {
      setpointStates[i] = modules[i].runOpenLoop(new SwerveModuleState(setpointStates[i].speedMetersPerSecond * 12.0 / MAX_LINEAR_SPEED, setpointStates[i].angle));

      // Convert voltage back to m/s
      // genuinely wtf is going on here
      setpointStates[i].speedMetersPerSecond *= MAX_LINEAR_SPEED / 12.0;
    }
  }
}
