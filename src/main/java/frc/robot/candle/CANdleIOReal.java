// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.candle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

/** Add your docs here. */
public class CANdleIOReal implements CANdleIO {
  public static final int LED_LENGTH = 59;
  private final CANdle candle;

  private final SolidColor solidColor = new SolidColor(0, LED_LENGTH);
  private final StrobeAnimation strobe = new StrobeAnimation(0, LED_LENGTH);

  public CANdleIOReal(int candleID, CANdleConfiguration config, CANBus canbus) {
    candle = new CANdle(candleID, canbus);
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.connected = candle.isConnected();
  }

  public void setSolid(RGBWColor color) {
    candle.setControl(solidColor.withColor(color));
  }

  public void setStrobe(RGBWColor color) {
    candle.setControl(strobe.withColor(color));
  }
}
