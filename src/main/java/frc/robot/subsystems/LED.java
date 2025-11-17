// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLED;
import frc.robot.RobotMap.mapLED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.hardware.CANdle;

public class LED extends SubsystemBase {
  CANdle LED = new CANdle(mapLED.LED_CAN);

  /** Creates a new LED. */

  public LED() {
    LED.getConfigurator().apply(constLED.LED_CONFIG);
  }

  public void setLED(int[] rgb) {
    if (rgb != null) {
      clearAnimation();
      LED.setControl(new SolidColor(constLED.LED_STRIP_START_INDEX, constLED.LED_NUMBER)
      .withColor(new RGBWColor(rgb[0], rgb[1], rgb[2], rgb[3])));
    }
  }

  public void setLEDStrobe(RGBWColor color) {
    clearAnimation();
    LED.setControl(new StrobeAnimation(constLED.LED_STRIP_START_INDEX, constLED.LED_NUMBER)
    .withColor(color));
  }

  public void clearAnimation() {
    for (int i = 0; i < 8; ++i) {
      LED.setControl(new EmptyAnimation(i));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
