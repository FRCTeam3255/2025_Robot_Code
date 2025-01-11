// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class ExampleAuto extends SequentialCommandGroup {
  Drivetrain subDrivetrain;

  /** Creates a new ExampleAuto. */
  public ExampleAuto(Drivetrain subDrivetrain) {
    /*
     * @formatter:off
     * 
     * Self-Driving Robot Workshop Usage:
     * 1. Build your self-driving robot command using the following commands:
     *  a. RotateInPlace
     *  b. Translate
     *  c. Commands.waitSeconds
     * TIP: You can hover over the names of these methods to view what inputs they need
     * An example is already done for you below, but feel free to remove it and get creative!
     *
     * 2. View a simulation of your robot driving!
     *  a. Press Ctrl+Shift+P or click the WPILib logo in the top right to open the command pallette. 
     *  b. Type or select "WPILib: Simulate Robot Code". 
     *  c. Once the code is done building, select "SIM GUI" from the pop-up window.
     *  d. Use Field2d or AdvantageScope to view your robot
     *  e. Run the autonomous by selecting "Autonomous" from Glass
     *  f. You can re-run the autonomous by selecting "Disable" and then hitting "Autonomous" again.
     * 
     * 2.5 Edit your simulation and run it again (if applicable)
     *  a. End the previous simulation by closing the SIM GUI window (red X in the top right)
     *      OR by returning to VSCode (here!) and hitting the Red Square at the top of your screen.
     *  b. Make your code changes
     *  b. Re-run the simulation by repeating the steps in part 2
     * 
     * 3. Run your path on the real robot
     *  a. Place the robot in our specified starting location
     *  b. Connect to the robot using a DriverStation laptop
     *  c. Run FRC DriverStation
     *  d. When you're ready, enable the robot in Autonomous
     * 
     * @formatter:on
     */

    addCommands(
        // -- Add your commands between these lines! --

        new Translate(
            subDrivetrain,
            Units.Meters.of(1.08),
            Units.Degrees.of(50),
            0.5),

        Commands.waitSeconds(0.5),

        new RotateInPlace(
            subDrivetrain,
            Units.Degrees.of(270)),

        new Translate(
            subDrivetrain,
            Units.Meters.of(1.08),
            Units.Degrees.of(180 + 50),
            0.3)

    // -- Add your commands between these lines! --
    );
  }
}