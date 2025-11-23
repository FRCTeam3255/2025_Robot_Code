// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLED;
import frc.robot.RobotMap.mapLED;

public class LED extends SubsystemBase {

  private final CANdle LED = new CANdle(mapLED.LED_CAN);

  public LED() {
    LED.getConfigurator().apply(constLED.LED_CONFIG);
  }

  public void setLEDSolid(RGBWColor color) {
    clearAnimations();
    LED.setControl(new SolidColor(constLED.LED_STRIP_START_INDEX, constLED.LED_NUMBER)
    .withColor(color));
  }

  public void setLEDMatrix(RGBWColor color, int LEDStartIndex, int LEDLength) {
    SolidColor matrixSolidColor = new SolidColor(LEDStartIndex, LEDLength)
    .withColor(color);
    LED.setControl(matrixSolidColor);
  }

  public void setLEDStrobe(RGBWColor color) {
    clearAnimations();
    LED.setControl(new StrobeAnimation(constLED.LED_STRIP_START_INDEX, constLED.LED_NUMBER)
    .withColor(color)
    .withSlot(0));
  }

  public void clearAnimations() {
    for (int i = 0; i < 8; ++i) {
        LED.setControl(new EmptyAnimation(i).withSlot(i));
    }
  }
}
