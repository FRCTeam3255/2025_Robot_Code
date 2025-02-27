// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rumbles;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class HasGamePieceRumble extends SequentialCommandGroup {
  public HasGamePieceRumble(SN_XboxController conDriver, SN_XboxController conOperator, RumbleType rumbleType,
      double rumbleIntensity) {
    addCommands(
        Commands.runOnce(() -> RobotContainer.justScored = false),
        Commands.runOnce(
            () -> conDriver.setRumble(rumbleType, rumbleIntensity))
            .alongWith(
                Commands.runOnce(() -> conOperator.setRumble(rumbleType, rumbleIntensity))),

        Commands.waitSeconds(0.5),

        Commands.runOnce(
            () -> conDriver.setRumble(rumbleType, 0))
            .alongWith(
                Commands.runOnce(() -> conOperator.setRumble(rumbleType, 0))));
  }
}
