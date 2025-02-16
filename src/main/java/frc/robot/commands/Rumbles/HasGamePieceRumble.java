// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rumbles;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HasGamePieceRumble extends SequentialCommandGroup {
  /** Creates a new HasCoralRumble. */
  public HasGamePieceRumble(SN_XboxController conDriver, SN_XboxController conOperator, RumbleType rumbleType,
      double rumbleIntensity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
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
