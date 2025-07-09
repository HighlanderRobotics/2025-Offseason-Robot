// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSubsystem extends SubsystemBase {
  // put constants here
  
  public enum ElevatorState {
    IDLE(0.0); //this will not be the real number!! this is just a placeholder

    private final double extensionMeters;

    private ElevatorState(double extensionMeters) {
      this.extensionMeters = extensionMeters;
    }

    public double getExtensionMeters() {
      return inputs.positionMeters;
    }
  }

  private ElevatorState state = ElevatorState.IDLE;

  //TODO: add IO layers
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // Sets the elevator to the height corresponding to the current state of the robot
    // Shoulder, intake, and any future subsystems should more or less follow this same pattern (additional logic might be needed though)
    setExtension(() -> state.extensionMeters);
  }

  public void setState(ElevatorState state) {
    this.state = state;
  }

  public Command setExtension(DoubleSupplier meters) {
    return Commands.none(); //TODO implement
  }
}
