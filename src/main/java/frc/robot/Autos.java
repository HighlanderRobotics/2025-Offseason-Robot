// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Autos {

      // Declare triggers
  @AutoLogOutput(key = "Superstructure/Auto Pre Score Request")
  public static Trigger autoPreScoreReq;

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq;

  @AutoLogOutput(key = "Superstructure/Auto Coral Intake Request")
  public static Trigger autoIntakeCoralReq;

  @AutoLogOutput(key = "Superstructure/Auto Algae Intake Request")
  public static Trigger autoIntakeAlgaeReq;
}
