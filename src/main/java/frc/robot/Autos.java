// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.CoralIntakeTarget;
import frc.robot.Robot.CoralScoreTarget;
import frc.robot.Robot.ScoringSide;
import frc.robot.Superstructure.SuperState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utils.AutoAim;
import frc.robot.utils.FieldUtils;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {

  private final SwerveSubsystem swerve;
  private final ArmSubsystem arm;
  private final AutoFactory factory;
  private Consumer<SuperState> stateSetter;

  // Declare triggers
  // mehhhhhhh
  private static boolean autoPreScore;
  private static boolean autoScore;
  private static boolean autoIntakeCoral;

  // private static boolean autoIntakeAlgae;

  @AutoLogOutput(key = "Superstructure/Auto Pre Score Request")
  public static Trigger autoPreScoreReq =
      new Trigger(() -> autoPreScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq =
      new Trigger(() -> autoScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Coral Intake Request")
  public static Trigger autoIntakeCoralReq =
      new Trigger(() -> autoIntakeCoral).and(DriverStation::isAutonomous);

  // @AutoLogOutput(key = "Superstructure/Auto Algae Intake Request")
  // public static Trigger autoIntakeAlgaeReq =
  //     new Trigger(() -> autoIntakeAlgae).and(DriverStation::isAutonomous);

  public enum PathEndType {
    INTAKE_CORAL_GROUND,
    INTAKE_CORAL_STACK,
    SCORE_CORAL,
    // INTAKE_ALGAE,
    // SCORE_ALGAE
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

    LOtoA4("LO", "A4", PathEndType.SCORE_CORAL),
    A4toSMM("A4", "SMM", PathEndType.INTAKE_CORAL_STACK),
    SMMtoB4("SMM", "B4", PathEndType.SCORE_CORAL),
    B4toSRL("B4", "SRL", PathEndType.INTAKE_CORAL_STACK),
    SRLtoC4("SRL", "C4", PathEndType.SCORE_CORAL),

    ROtoB4("RO", "B4", PathEndType.SCORE_CORAL),
    B4toSMM("B4", "SMM", PathEndType.INTAKE_CORAL_STACK),
    SMMtoA4("SMM", "A4", PathEndType.SCORE_CORAL),
    A4toSLR("A4", "SLR", PathEndType.INTAKE_CORAL_STACK),
    SLRtoL4("SLR", "L4", PathEndType.SCORE_CORAL),

    CMtoH4("CM", "H4", PathEndType.SCORE_CORAL),
  // H4toGH("H4", "GH", PathEndType.INTAKE_ALGAE),
  // GHtoBR("GH", "BR", PathEndType.SCORE_ALGAE),
  // BRtoIJ("BR", "IJ", PathEndType.INTAKE_ALGAE),
  // IJtoBR("IJ", "BR", PathEndType.SCORE_ALGAE)
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

  public Autos(SwerveSubsystem swerve, ArmSubsystem arm, Consumer<SuperState> stateSetter) {
    this.swerve = swerve;
    this.arm = arm;
    this.stateSetter = stateSetter;
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
    Path[] paths = {Path.LOtoA4, Path.A4toSMM, Path.SMMtoB4, Path.B4toSRL, Path.SRLtoC4};

    Command autoCommand =
        Commands.runOnce(
                () -> {
                  stateSetter.accept(SuperState.READY_CORAL_ARM);
                  arm.setHasCoralForAuto(true);
                })
            .andThen(paths[0].getTrajectory(routine).resetOdometry());

    for (Path path : paths) {
      autoCommand =
          autoCommand.andThen(
              Commands.print("Running path: " + path.toString()).andThen(runPath(path, routine)));
    }

    routine
        .active()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Robot.setCoralScoreTarget(CoralScoreTarget.L4);
                  Robot.setCoralIntakeTarget(CoralIntakeTarget.STACK);
                }))
        .whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getRightStackAuto() {
    final AutoRoutine routine = factory.newRoutine("Left Stack Auto");
    bindCoralElevatorExtension(routine);
    Path[] paths = {Path.ROtoB4, Path.B4toSMM, Path.SMMtoA4, Path.A4toSLR, Path.SLRtoL4};

    Command autoCommand =
        Commands.runOnce(() -> stateSetter.accept(SuperState.READY_CORAL_ARM))
            .andThen(paths[0].getTrajectory(routine).resetOdometry());

    for (Path path : paths) {
      autoCommand = autoCommand.andThen(runPath(path, routine));
    }

    routine
        .active()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Robot.setCoralScoreTarget(CoralScoreTarget.L4);
                  Robot.setCoralIntakeTarget(CoralIntakeTarget.STACK);
                }))
        .whileTrue(autoCommand);

    return routine.cmd();
  }

  public Command getAlgaeAuto() {
    final AutoRoutine routine = factory.newRoutine("Algae auto");
    // 4.5 should make the elevator extend earlier so it doesn't knock the algae off the reef (in
    // theory)
    bindCoralElevatorExtension(routine, 4.5); // TODO: TUNE
    Path[] paths = {Path.CMtoH4}; // Path.GHtoBR, Path.BRtoIJ, Path.IJtoBR};

    Command autoCommand =
        Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> stateSetter.accept(SuperState.READY_CORAL_ARM)),
                Commands.runOnce(() -> arm.hasCoral = true)),
            Commands.runOnce(() -> Robot.setScoringSide(ScoringSide.LEFT)),
            paths[0].getTrajectory(routine).resetOdometry(),
            runPath(paths[0], routine));
    // intakeAlgaeInAuto(() -> paths[0].getTrajectory(routine).getFinalPose().get()),
    // runPath(paths[1], routine),
    // runPath(paths[2], routine),
    // runPath(paths[3], routine));

    routine
        .active()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Robot.setCoralScoreTarget(CoralScoreTarget.L4);
                  // Robot.setAlgaeIntakeTarget(AlgaeIntakeTarget.HIGH);
                }))
        .whileTrue(autoCommand);

    // routine
    //     .observe(paths[3].getTrajectory(routine).active());
    // .onTrue(Commands.runOnce(() -> Robot.setAlgaeIntakeTarget(AlgaeIntakeTarget.LOW)));

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
        return runPathThenIntakeStackCoral(path, routine);
        // case SCORE_ALGAE:
        //   return runPathThenScoreAlgae(path, routine);
        // case INTAKE_ALGAE:
        //   return runPathThenIntakeAlgae(path, routine);
      default: // TODO this should never happen?
        return Commands.none();
    }
  }

  // Only for barge rn. Is this a problem?? I assume no...
  // public Command runPathThenScoreAlgae(Path path, AutoRoutine routine) {
  //   return Commands.sequence(
  //       path.getTrajectory(routine)
  //           .cmd()
  //           .until(
  //               routine.observe(
  //                   path.getTrajectory(routine)
  //                       .atTime(path.getTrajectory(routine).getRawTrajectory().getTotalTime()))),
  //       scoreAlgaeBargeInAuto());
  // }

  // public Command scoreAlgaeBargeInAuto() {
  //   return Commands.race(
  //       swerve.autoAimToBarge(() -> 0),
  //       Commands.sequence(
  //           Commands.runOnce(() -> Autos.autoPreScore = true),
  //           Commands.waitUntil(swerve::nearBarge),
  //           setAutoScoreReqTrue(),
  //           Commands.runOnce(() -> arm.setSimAlgae(false)),
  //           Commands.waitUntil(() -> !arm.hasAlgae),
  //           // Also sets prescore false
  //           setAutoScoreReqFalse()));
  // }

  public Command runPathThenScoreCoral(Path path, AutoRoutine routine) {
    return Commands.sequence(
        Commands.print("Run path than score coral" + path.name()),
        path.getTrajectory(routine)
            .cmd()
            .until(
                routine.observe(
                    path.getTrajectory(routine)
                        .atTime(
                            path.getTrajectory(routine).getRawTrajectory().getTotalTime() - 0.3))),
        Commands.print("Done running path"),
        scoreCoralInAuto(() -> path.getTrajectory(routine).getFinalPose().get()));
  }

  public Command scoreCoralInAuto(Supplier<Pose2d> trajEndPose) {
    return Commands.sequence(
            Commands.waitUntil(
                new Trigger(
                        () -> {
                          boolean isInTolerance =
                              swerve.isInAutoAimTolerance(
                                  Robot.getCoralScoreTarget().equals(Robot.CoralScoreTarget.L4)
                                      ? FieldUtils.CoralTargets.getClosestTargetL4(
                                          trajEndPose.get())
                                      : FieldUtils.CoralTargets.getClosestTargetL23(
                                          trajEndPose.get()));
                          Logger.recordOutput("AutoAim/Is in tolerance", isInTolerance);
                          return isInTolerance;
                        })
                    .and(swerve::isNotMoving)
                    .debounce(0.06 * 2)),
            setAutoScoreReqTrue(),
            Commands.runOnce(() -> autoPreScore = false),
            // arm.setSimCoral(() -> false),
            Commands.runOnce(() -> arm.setSimCoral(false)),
            waitUntilNoCoral(),
            setAutoScoreReqFalse())
        .raceWith(
            swerve.translateToPose(
                () ->
                    Robot.getCoralScoreTarget().equals(CoralScoreTarget.L4)
                        ? FieldUtils.CoralTargets.getClosestTargetL4(trajEndPose.get())
                        : FieldUtils.CoralTargets.getClosestTargetL23(trajEndPose.get()),
                () -> new ChassisSpeeds(),
                new Constraints(1.5, 1.0),
                AutoAim.DEFAULT_ANGULAR_CONSTRAINTS));
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
        intakeGroundCoralInAuto(() -> path.getTrajectory(routine).getFinalPose()));
  }

  // bruh why was i inconsistent on this
  // TODO intakeCoralInAuto (cause ground/stack intake)
  public Command intakeGroundCoralInAuto(Supplier<Optional<Pose2d>> pose) {
    return Commands.none();
  }

  // TODO: SHOULD THIS AUTOALIGN??
  public Command runPathThenIntakeStackCoral(Path path, AutoRoutine routine) {
    return Commands.sequence(
        path.getTrajectory(routine).cmd().until(routine.idle()), intakeStackCoralInAuto());
  }

  public Command intakeStackCoralInAuto() {
    return Commands.sequence(
        Commands.runOnce(() -> autoIntakeCoral = true),
        Commands.waitUntil(arm::hasGamePiece),
        Commands.runOnce(() -> autoIntakeCoral = false));
  }

  // public Command runPathThenIntakeAlgae(Path path, AutoRoutine routine) {
  //   return Commands.sequence(
  //       path.getTrajectory(routine).cmd(),
  //       intakeAlgaeInAuto(() -> path.getTrajectory(routine).getFinalPose().get()));
  // }

  // public Command intakeAlgaeInAuto(Supplier<Pose2d> trajEndPose) {
  //   return Commands.sequence(
  //       swerve.autoAimToOffsetAlgaePose().until(swerve::nearIntakeAlgaeOffsetPose),
  //       Commands.runOnce(() -> autoIntakeAlgae = true),
  //       swerve
  //           .approachAlgae()
  //           .until(arm::hasGamePiece)
  //           .alongWith(
  //               Commands.either(
  //                   Commands.sequence(
  //                       Commands.waitSeconds(1), Commands.runOnce(() -> arm.setSimAlgae(true))),
  //                   Commands.none(),
  //                   Robot::isSimulation)),
  //       Commands.runOnce(() -> autoIntakeAlgae = false));
  // }

  public Command setAutoScoreReqTrue() {
    return Commands.runOnce(
        () -> {
          autoScore = true;
        });
  }

  public Command setAutoPreScoreReqTrue() {
    return Commands.runOnce(() -> autoPreScore = true);
  }

  public Command setAutoScoreReqFalse() {
    return Commands.runOnce(
        () -> {
          autoScore = false;
          autoPreScore = false;
        });
  }

  public Command waitUntilNoCoral() {
    return Commands.waitUntil(() -> !arm.hasGamePiece());
  }
}
