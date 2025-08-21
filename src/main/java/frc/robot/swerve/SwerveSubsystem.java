package frc.robot.swerve;

import javax.print.attribute.standard.PrinterURI;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.constants.SwerveConstants;
import frc.robot.swerve.module.Module;
import frc.robot.swerve.module.ModuleIOReal;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveConstants swerveConstants;

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right

  private SwerveDriveKinematics kinematics;

  public SwerveSubsystem(SwerveConstants swerveConstants) {
    // TODO: MAKE THESE WORK FOR SIM AS WELL

    modules = new Module[] {
      new Module(new ModuleIOReal(swerveConstants.getFrontLeftModule())),
      new Module(new ModuleIOReal(swerveConstants.getFrontRightModule())),
      new Module(new ModuleIOReal(swerveConstants.getBackLeftModule())),
      new Module(new ModuleIOReal(swerveConstants.getBackRightModule()))
    };

  }

  @Override
  public void periodic() {

      // Updates each module
      for (Module module : modules) {
        module.periodic();
      }
  }

  private void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Converts time continuous chassis speeds to setpoints after the specified time (dtSeconds)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert drivetrain setpoint into individual module setpoints
    final SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];

    for (int i = 0; i < optimizedStates.length; i++) {
      if (openLoop) {
        optimizedStates[i] = modules[i].runOpenLoop(states[i], true);
      } else {
        optimizedStates[i] = modules[i].runClosedLoop(states[i]);
      }

    }

  }

  
}
