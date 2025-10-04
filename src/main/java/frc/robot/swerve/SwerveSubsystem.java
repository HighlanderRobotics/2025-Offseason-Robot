// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AutoAim;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveSubsystem extends SubsystemBase {

  // we don't have multiple robots so i kinda don't care
  public static final SwerveConstants constants = new SwerveConstants();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    // return estimator.getEstimatedPosition();
    return Pose2d.kZero;
  }

  // public boolean isNearPose(Pose2d pose) {
  //   return
  // }

  public boolean isNearReef() {
    return getPose()
            .getTranslation()
            .minus(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? AutoAim.BLUE_REEF_CENTER
                    : AutoAim.RED_REEF_CENTER)
            .getNorm()
        < 3.25;
  }
}
