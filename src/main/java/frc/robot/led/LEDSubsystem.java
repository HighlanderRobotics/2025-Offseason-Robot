// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  public static final int LED_LENGTH = 59; // no one knows what this number is lmaoooo
  public static final int LED_ID = 0;

  public static final Color PURPLE = new Color("#A000D0");

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
  private double rainbowStart = 0;
  private double dashStart = 0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDIO io) {
    this.io = io;
    io.solid(Color.kPurple);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  private void setIndex(int i, Color color) {
    io.set(i, color);
  }

  private void setSolid(Color color) {
    io.solid(color);
  }

  public Command setSolidCmd(Color color) {
    return this.run(() -> setSolid(color));
  }

  public Command setBlinkingCmd(
      Supplier<Color> onColor, Supplier<Color> offColor, double frequency) {
    return Commands.repeatingSequence(
        setSolidCmd(onColor.get()).withTimeout(1.0 / frequency),
        setSolidCmd(offColor.get()).withTimeout(1.0 / frequency));
  }

  public Command setSplitCmd(Color upper, Color lower) {
    return this.run(
        () -> {
          for (int i = 0; i < LED_LENGTH; i++) {
            io.set(i, i < LED_LENGTH / 2 ? lower : upper);
          }
        });
  }

  public Command setBlinkingSplitCmd(
      Supplier<Color> upOnColor, Supplier<Color> downOnColor, double frequency) {
    return Commands.repeatingSequence(
        setSplitCmd(upOnColor.get(), downOnColor.get()).withTimeout(1.0 / frequency),
        setSolidCmd(Color.kBlack).withTimeout(1.0 / frequency));
  }

  /** Sets the first portion of the leds to a color, and the rest off */
  public Command setProgressCmd(Color color, DoubleSupplier progress) {
    return this.run(
        () -> {
          for (int i = 0; i < LED_LENGTH; i++) {
            setIndex(i, i < progress.getAsDouble() * LED_LENGTH ? color : Color.kBlack);
          }
        });
  }

  public Command setRainbowCmd() {
    return this.run(
        () -> {
          for (int i = 0; i < LED_LENGTH; i++) {
            setIndex(i, Color.fromHSV((int) rainbowStart % 180 + i, 255, 255));
          }
          rainbowStart += 6;
        });
  }

  public Command setRunAlongCmd(
      Supplier<Color> colorDash, Color colorBg, int dashLength, double frequency) {
    return this.run(
        () -> {
          setSolid(colorBg);
          for (int i = (int) dashStart; i < dashStart + dashLength; i++) {
            setIndex(i % LED_LENGTH, colorDash.get());
          }

          dashStart += LED_LENGTH * frequency * 0.020;
          dashStart %= LED_LENGTH;
        });
  }
}
