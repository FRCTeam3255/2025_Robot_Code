// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/** Add your docs here. */

@CustomLoggerFor(Robot.class)
public class AllianceLogger extends ClassSpecificLogger<Robot> {
  public AllianceLogger() {
    super(Robot.class);
  }

  public void update(EpilogueBackend backend, Robot robot) {
    backend.log("Alliance", DriverStation.getAlliance().toString());

  }

}
