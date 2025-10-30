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
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.CoralIntakeTarget;
import frc.robot.Robot.CoralScoreTarget;
import frc.robot.arm.ArmSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  public static final double ARM_PC_CURRENT_THRESHOLD = 20.0;
  public static final double ElEVATOR_PC_CURRENT_THRESHOLD = 20.0;
  public static final double INTAKE_PC_CURRENT_THRESHOLD = 20.0;

  private final SwerveSubsystem swerve;
  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;
  private final IntakeSubsystem intake;
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
    BtoPRO("B", "PRO", PathEndType.INTAKE_CORAL_GROUND);

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

  public Autos(
      SwerveSubsystem swerve,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake) {
    this.swerve = swerve;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
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
        .observe(arm::hasGamePiece)
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
                new Trigger(() -> swerve.isNearPoseAuto(trajEndPose))
                    .and(swerve::isNotMoving)
                    .debounce(0.06 * 2)),
            setAutoScoreReqTrue(),
            waitUntilNoGamePiece(),
            setAutoScoreReqFalse())
        .raceWith(swerve.autoAimAuto(trajEndPose));
  }

  // bruh why was i inconsistent on this
  // TODO intakeCoralInAuto (cause ground/stack intake)
  public Command intakeCoralInAuto(Supplier<Optional<Pose2d>> pose) {
    return Commands.none();
  }

  public Command setAutoIntakeCoralReqTrue() {
    return Commands.runOnce(
        () -> {
          autoIntakeCoral = true;
        });
  }

  public Command setAutoIntakeCoralReqFalse() {
    return Commands.runOnce(
        () -> {
          autoIntakeCoral = false;
        });
  }

  public Command setAutoIntakeAlgaeReqTrue() {
    return Commands.runOnce(
        () -> {
          autoIntakeAlgae = true;
        });
  }

  public Command setAutoIntakeAlgaeReqFalse() {
    return Commands.runOnce(
        () -> {
          autoIntakeAlgae = false;
        });
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

  public Command setSimHasCoralTrue() {
    return Robot.isSimulation() ? Commands.runOnce(() -> arm.setSimCoral(true)) : Commands.none();
  }

  public Command waitUntilNoGamePiece() {
    return Commands.waitUntil(() -> !arm.hasGamePiece()).alongWith(setSimHasCoralFalse());
  }

  public Command waitUntilHasGamePiece() {
    return Commands.waitUntil(() -> arm.hasGamePiece()).alongWith(setSimHasCoralTrue());
  }

  // TODO fix names?
  // these are differnet than the other intake/score stuff because they dont need the pose stuff
  // are there current checks for intake as well?
  public Command intakeCoralAutoPC(CoralIntakeTarget target) {
    return Commands.runOnce(
            () -> {
              Robot.setCoralIntakeTarget(target);
            })
        .andThen(
            Commands.sequence(
                    setAutoIntakeCoralReqTrue(),
                    waitUntilHasGamePiece(),
                    setAutoIntakeCoralReqFalse())
                .raceWith(
                    Commands.runOnce(
                        () -> {
                          if (intake.getFilteredStatorCurrentAmps() > INTAKE_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert intake current spike at coral" + target,
                                intake.getFilteredStatorCurrentAmps());
                        })));
  }

  public Command intakeAlgaeAutoPC(AlgaeIntakeTarget target) {
    return Commands.runOnce(
            () -> {
              Robot.setAlgaeIntakeTarget(target);
            })
        .andThen(
            Commands.sequence(
                    setAutoIntakeAlgaeReqTrue(),
                    waitUntilHasGamePiece(),
                    setAutoIntakeAlgaeReqFalse())
                .raceWith(
                    Commands.runOnce(
                        () -> {
                          if (intake.getFilteredStatorCurrentAmps() > INTAKE_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert intake current spike at algae" + target,
                                intake.getFilteredStatorCurrentAmps());
                        })));
  }

  public Command scoreCoralAutoPC(CoralScoreTarget level, double wait) {
    return Commands.runOnce(
            () -> {
              Robot.setCoralScoreTarget(level);
            })
        .andThen(
            Commands.sequence(
                    Commands.waitSeconds(wait),
                    setAutoScoreReqTrue(),
                    waitUntilNoGamePiece(),
                    setAutoScoreReqFalse())
                .raceWith(
                    Commands.runOnce(
                        () -> {
                          // TODO belt skip checking?
                          if (arm.getFilteredStatorCurrentAmps() > ARM_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert arm current spike at coral level" + level,
                                arm.getFilteredStatorCurrentAmps());
                          if (elevator.currentFilterValue > ElEVATOR_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert elevator current spike at coral level"
                                    + level,
                                elevator.currentFilterValue);
                        }))
                .repeatedly());
  }

  public Command scoreAlgaeAutoPC(AlgaeScoreTarget level, double wait) {
    return Commands.runOnce(
            () -> {
              Robot.setAlgaeScoreTarget(level);
            })
        .andThen(
            Commands.sequence(
                    Commands.waitSeconds(wait),
                    setAutoScoreReqTrue(),
                    waitUntilNoGamePiece(),
                    setAutoScoreReqFalse())
                .raceWith(
                    Commands.runOnce(
                        () -> {
                          // TODO threshold may be different for algae scoring
                          // TODO belt skip checking?
                          if (arm.getFilteredStatorCurrentAmps() > ARM_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert arm current spike at algae" + level,
                                arm.getFilteredStatorCurrentAmps());
                          if (elevator.currentFilterValue > ElEVATOR_PC_CURRENT_THRESHOLD)
                            Logger.recordOutput(
                                "Autos/pitchecks alert elevator current spike at algae" + level,
                                elevator.currentFilterValue);
                        }))
                .repeatedly());
  }

  public Command pitCheck() {
    return Commands.sequence(
        // coral stuff
        intakeCoralAutoPC(CoralIntakeTarget.GROUND),
        scoreCoralAutoPC(CoralScoreTarget.L1, 0.5),
        intakeCoralAutoPC(CoralIntakeTarget.GROUND),
        scoreCoralAutoPC(CoralScoreTarget.L2, 0.50),
        intakeCoralAutoPC(CoralIntakeTarget.STACK),
        scoreCoralAutoPC(CoralScoreTarget.L3, 0.50),
        intakeCoralAutoPC(CoralIntakeTarget.STACK),
        scoreCoralAutoPC(CoralScoreTarget.L4, 0.50),
        // algae stuff
        intakeAlgaeAutoPC(AlgaeIntakeTarget.GROUND),
        scoreAlgaeAutoPC(AlgaeScoreTarget.PROCESSOR, 0.5),
        intakeAlgaeAutoPC(AlgaeIntakeTarget.STACK),
        scoreAlgaeAutoPC(AlgaeScoreTarget.PROCESSOR, 0.5),
        intakeAlgaeAutoPC(AlgaeIntakeTarget.LOW),
        scoreAlgaeAutoPC(AlgaeScoreTarget.BARGE, 0.5),
        intakeAlgaeAutoPC(AlgaeIntakeTarget.HIGH),
        scoreAlgaeAutoPC(AlgaeScoreTarget.BARGE, 0.5));
  }
}
