// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLED;
import frc.robot.RobotMap.mapLED;

public class LED extends SubsystemBase {
  private final CANdle LED = new CANdle(mapLED.LED_CAN);
  private final SolidColor solidColor = new SolidColor(constLED.LED_STRIP_START_INDEX, constLED.LED_NUMBER);

  public LED() {
    var cfg = new CANdleConfiguration();
    LED.getConfigurator().apply(cfg);
  }

  public Command solidColor(RGBWColor color) {
    return run(() -> setSolidColor(color));
  }

  public void setSolidColor(RGBWColor color) {
    setControl(solidColor.withColor(color));
  }

  private void setControl(ControlRequest control) {
    LED.setControl(control);
  }
}

