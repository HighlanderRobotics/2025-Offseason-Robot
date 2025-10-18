// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.CoralIntakeTarget;
import frc.robot.Robot.CoralScoreTarget;
import frc.robot.arm.ArmSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class Autos {

  private final SwerveSubsystem swerve;
  private final ArmSubsystem arm;
  private final AutoFactory factory;

  // Declare triggers
  // mehhhhhhh
  private static boolean autoPreScore;
  private static boolean autoScore;
  private static boolean autoIntakeCoral;
  private static boolean autoIntakeAlgae;

  @AutoLogOutput(key = "Superstructure/Auto Pre Score Request")
  public static Trigger autoPreScoreReq = new Trigger(() -> autoPreScore);

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq = new Trigger(() -> autoScore);

  @AutoLogOutput(key = "Superstructure/Auto Coral Intake Request")
  public static Trigger autoIntakeCoralReq = new Trigger(() -> autoIntakeCoral);

  @AutoLogOutput(key = "Superstructure/Auto Algae Intake Request")
  public static Trigger autoIntakeAlgaeReq = new Trigger(() -> autoIntakeAlgae);

  public enum PathEndType {
    INTAKE_CORAL_GROUND,
    INTAKE_CORAL_STACK,
    SCORE_CORAL,
    INTAKE_ALGAE,
    SCORE_ALGAE
  }

  public enum Path {
    ROtoE("RO", "E", PathEndType.SCORE_CORAL),
    EtoPRO("E", "PRO", PathEndType.INTAKE_CORAL_GROUND),
    PROtoD("PRO", "D", PathEndType.SCORE_CORAL),
    DtoPRO("D", "PRO", PathEndType.INTAKE_CORAL_GROUND),
    PROtoC("PRO", "C", PathEndType.SCORE_CORAL),
    CtoPRM("C", "PRM", PathEndType.INTAKE_CORAL_GROUND),
    PRMtoB("PRM", "B", PathEndType.SCORE_CORAL),
    BtoPRO("B", "PRO", PathEndType.INTAKE_CORAL_GROUND),
    
    LOtoI4("LO", "I4", PathEndType.SCORE_CORAL),
    I4toSLM("I4", "SLM", PathEndType.INTAKE_CORAL_STACK),
    SLMtoL4("SLM", "L4", PathEndType.SCORE_CORAL),
    L4toSML("L4", "SML", PathEndType.INTAKE_CORAL_STACK),
    SMLtoA4("SML", "A4", PathEndType.SCORE_CORAL),
    A4toSRL("A4", "SRL", PathEndType.INTAKE_CORAL_STACK),
    SRLtoC4("SRL", "C4", PathEndType.SCORE_CORAL)
    ;

    private final String start;
    private final String end;
    private final PathEndType type;

    private Path(String start, String end, PathEndType type) {
      this.start = start;
      this.end = end;
      this.type = type;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(start + "to" + end);
    }
  }

  public Autos(SwerveSubsystem swerve, ArmSubsystem arm) {
    this.swerve = swerve;
    this.arm = arm;
    factory =
        new AutoFactory(
            swerve::getPose, swerve::resetPose, swerve.choreoDriveController(), true, swerve
            // ,
            // (traj, edge) -> {
            //   if (Robot.ROBOT_TYPE != RobotType.REAL)
            //     Logger.recordOutput(
            //         "Choreo/Active Traj",
            //         DriverStation.getAlliance().isPresent()
            //                 && DriverStation.getAlliance().get().equals(Alliance.Blue)
            //             ? traj.getPoses()
            //             : traj.flipped().getPoses());
            // }
            );
  }

  public Command getLeftStackAuto() {
    final AutoRoutine routine = factory.newRoutine("Left Stack Auto");
    bindCoralElevatorExtension(routine);
    Path[] paths = {
      Path.LOtoI4,
      Path.I4toSLM,
      Path.SLMtoL4,
      Path.L4toSML,
      Path.SMLtoA4,
      Path.A4toSRL,
      Path.SRLtoC4
    };

    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path path : paths) {
      autoCommand = autoCommand.andThen(runPath(path, routine));
    }

    routine.active()
      .onTrue(Commands.runOnce(() -> {
        Robot.setCoralScoreTarget(CoralScoreTarget.L4);
        Robot.setCoralIntakeTarget(CoralIntakeTarget.STACK);
      }))
      .whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getRightOutsideAuto() {
    final AutoRoutine routine = factory.newRoutine("RO to E");
    bindCoralElevatorExtension(routine);
    Path[] paths = {
      Path.ROtoE,
      Path.EtoPRO,
      Path.PROtoD,
      Path.DtoPRO,
      Path.PROtoC,
      Path.CtoPRM,
      Path.PRMtoB,
      Path.BtoPRO
    };
    // Will always need to reset odo at the start of a routine
    Command autoCommand = paths[0].getTrajectory(routine).resetOdometry();

    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }

    routine
        .active()
        .onTrue(Commands.runOnce(() -> Robot.setCoralScoreTarget(CoralScoreTarget.L4)))
        .whileTrue(autoCommand);

    routine
        .observe(paths[5].getTrajectory(routine).done())
        .onTrue(Commands.runOnce(() -> Robot.setCoralScoreTarget(CoralScoreTarget.L2)));

    return routine.cmd();
  }

  public void bindCoralElevatorExtension(AutoRoutine routine) {
    bindCoralElevatorExtension(routine, 4); // TODO tune
  }

  public void bindCoralElevatorExtension(AutoRoutine routine, double toleranceMeters) {
    routine
        .observe(arm::hasCoral)
        .and(() -> swerve.isNearReef(toleranceMeters))
        .whileTrue(Commands.run(() -> autoPreScore = true))
        .whileFalse(Commands.run(() -> autoPreScore = false));
  }

  public Command runPath(Path path, AutoRoutine routine) {
    PathEndType type = path.type;
    switch (type) {
      case SCORE_CORAL:
        return runPathThenScoreCoral(path, routine);
      case INTAKE_CORAL_GROUND:
        return runPathThenIntakeCoralGround(path, routine);
      case INTAKE_CORAL_STACK:
        return null; // lol
      case SCORE_ALGAE:
        return null; // lol
      case INTAKE_ALGAE:
        return null; // lol
      default: // TODO this should never happen?
        return Commands.none();
    }
  }

  public Command runPathThenScoreCoral(Path path, AutoRoutine routine) {
    return Commands.sequence(
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                                - (path.end.length() == 1 ? 0.3 : 0.0)))),
        scoreCoralInAuto(() -> path.getTrajectory(routine).getFinalPose().get()));
  }

  public Command runPathThenIntakeCoralGround(Path path, AutoRoutine routine) {
    return Commands.sequence(
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime()
                                - (path.end.length() == 1 ? 0.3 : 0.0)))),
        intakeCoralInAuto(() -> path.getTrajectory(routine).getFinalPose()));
  }

  public Command scoreCoralInAuto(Supplier<Pose2d> trajEndPose) {
    return Commands.sequence(
            Commands.waitUntil(
                new Trigger(() -> swerve.isNearPoseAuto(trajEndPose.get()))
                    .and(swerve::isNotMoving)
                    .debounce(0.06 * 2)),
            setAutoScoreReqTrue(),
            waitUntilNoCoral(),
            setAutoScoreReqFalse())
        .raceWith(swerve.autoAimAuto(trajEndPose));
  }

  // bruh why was i inconsistent on this
  // TODO intakeCoralInAuto (cause ground/stack intake)
  public Command intakeCoralInAuto(Supplier<Optional<Pose2d>> pose) {
    return Commands.none();
  }

  public Command setAutoScoreReqTrue() {
    return Commands.runOnce(
        () -> {
          autoScore = true;
        });
  }

  public Command setAutoScoreReqFalse() {
    return Commands.runOnce(
        () -> {
          autoScore = false;
          autoPreScore = false;
        });
  }

  public Command setSimHasCoralFalse() {
    return Robot.isSimulation() ? Commands.runOnce(() -> arm.setSimCoral(false)) : Commands.none();
  }

  public Command waitUntilNoCoral() {
    return Commands.waitUntil(() -> !arm.hasCoral()).alongWith(setSimHasCoralFalse());
  }
}
