// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLED;
import frc.robot.RobotMap.mapLED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class LED extends SubsystemBase {
  CANdle LED;

  /** Creates a new LED. */

  public LED() {
    LED = new CANdle(mapLED.LED_CAN);
    configure();
  }

  public void configure() {
    LED.configFactoryDefault();
    LED.configBrightnessScalar(constLED.LED_BRIGHTNESS);
  }

  public void setLED(int[] rgb) {
    LED.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  public void setLEDAnimation(Animation animation, int animationSlot) {
    LED.animate(animation, animationSlot);
  }

  public void setLEDMatrix(int[] rgb, int LEDStartIndex, int LEDLength) {
    LED.setLEDs(rgb[0], rgb[1], rgb[2], 0, LEDStartIndex, LEDLength);
  }

  public void clearAnimation() {
    LED.clearAnimation(0);
    LED.clearAnimation(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
