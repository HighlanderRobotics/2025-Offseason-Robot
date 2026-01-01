// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Superstructure;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  public static final int LED_LENGTH = 59; // no one knows what this number is lmaoooo
  public static final int LED_ID = 0;

  public static final Color PURPLE = new Color("#A000D0");

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
  private double rainbowStart = 0;
  private double dashStart = 0;
  private BooleanSupplier atExtension; // this is totally stupid

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDIO io, BooleanSupplier atExtension) {
    this.io = io;
    this.atExtension = atExtension;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);

    if (DriverStation.isDisabled()) {
      runAlong(
          DriverStation.getAlliance()
              .map((a) -> a == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kWhite),
          PURPLE,
          4,
          1.0);
    } else {
      switch (Superstructure.getState()) {
        case IDLE:
          solid(PURPLE);
          break;
        case INTAKE_CORAL_GROUND:
          blink(Color.kWhite, PURPLE, 0.1);
          break;
        case READY_CORAL_INTAKE:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 2);
          blink(
              Color.kWhite,
              Robot.getCoralScoreTarget().getColor(),
              0.5,
              LED_LENGTH / 2,
              LED_LENGTH);
          // yellow lime lightblue for location in intake
          // green teal blue purple for scoring level
          // orange and pink for scoring side
          break;
        case PRE_PRE_HANDOFF:
          blink(PURPLE, Color.kBlack, 0.1);
          break;
        case PRE_HANDOFF_RIGHT:
        case HANDOFF_RIGHT:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 3);
          solid(
              atExtension.getAsBoolean() ? Color.kGreen : Color.kRed,
              LED_LENGTH / 3,
              2 * LED_LENGTH / 3);
          blink(
              Color.kYellow,
              Robot.getCoralScoreTarget().getColor(),
              0.5,
              2 * LED_LENGTH / 3,
              LED_LENGTH);
          break;
        case PRE_HANDOFF_CENTER:
        case HANDOFF_CENTER:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 2);
          blink(
              Color.kLime, Robot.getCoralScoreTarget().getColor(), 0.5, LED_LENGTH / 2, LED_LENGTH);
          break;
        case PRE_HANDOFF_LEFT:
        case HANDOFF_LEFT:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 2);
          blink(
              Color.kDeepSkyBlue,
              Robot.getCoralScoreTarget().getColor(),
              0.5,
              LED_LENGTH / 2,
              LED_LENGTH);
          break;
        case POST_HANDOFF:
          blink(PURPLE, Color.kBlack, 0.1);
          break;
        case PRE_L1:
        case PRE_L2:
        case PRE_L3:
        case PRE_L4:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 2);
          blink(Robot.getCoralScoreTarget().getColor(), PURPLE, 0.1, LED_LENGTH / 2, LED_LENGTH);
        case SCORE_L1:
        case SCORE_L2:
        case SCORE_L3:
        case SCORE_L4:
          solid(Robot.getScoringSide().getColor(), 0, LED_LENGTH / 2);
          blink(Robot.getCoralScoreTarget().getColor(), PURPLE, 0.5, LED_LENGTH / 2, LED_LENGTH);
          break;
        case PRE_CLIMB:
          blink(Color.kMagenta, PURPLE, 0.1);
          break;
        case CLIMB:
          blink(Color.kMagenta, PURPLE, 0.5);
          break;
        default:
          solid(Color.kBlack);
      }
    }
  }

  private void setIndex(int i, Color color) {
    io.set(i, color);
  }

  private void solid(Color color) {
    io.solid(color);
  }

  private void solid(Color color, int start, int end) {
    for (int i = start; i < end; i++) {
      setIndex(i, color);
    }
  }

  public void blink(Color onColor, Color offColor, double duration) {
    blink(onColor, offColor, duration, 0, LED_LENGTH);
  }

  public void blink(Color onColor, Color offColor, double duration, int start, int end) {
    if (Timer.getTimestamp() % (2 * duration) < duration) {
      solid(onColor, start, end);
    } else {
      solid(offColor, start, end);
    }
  }

  public void runAlong(Color colorDash, Color colorBg, int dashLength, double frequency) {
    solid(colorBg);
    for (int i = (int) dashStart; i < dashStart + dashLength; i++) {
      setIndex(i % LED_LENGTH, colorDash);
    }
    dashStart += LED_LENGTH * frequency * 0.020;
    dashStart %= LED_LENGTH;
  }

  public Command blinkCmd(Color onColor, Color offColor, double duration) {
    return Commands.run(() -> blink(onColor, offColor, duration)).repeatedly();
  }
}
