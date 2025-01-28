// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constField;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  boolean hasAutonomousRun = false;
  private boolean bothSubsystemsZeroed = false;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Epilogue.bind(this);
    m_robotContainer = new RobotContainer();

    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("logs");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(Constants.constControllers.SILENCE_JOYSTICK_WARNINGS);
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.AddVisionMeasurement().schedule();
    m_robotContainer.updateLoggedPoses();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    bothSubsystemsZeroed = m_robotContainer.allZeroed();
    m_robotContainer.setMegaTag2(false);

    if (!hasAutonomousRun) {
      m_robotContainer.checkForManualZeroing().schedule();
    }
  }

  @Override
  public void disabledPeriodic() {
    constField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", constField.ALLIANCE.toString());
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMegaTag2(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (bothSubsystemsZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy(() -> m_autonomousCommand).schedule();
    } else if (m_autonomousCommand != null) {
      m_robotContainer.zeroSubsystems().andThen(Commands.deferredProxy(() -> m_autonomousCommand)).schedule();
    } else {
      m_robotContainer.zeroSubsystems().schedule();
    }

    hasAutonomousRun = true;
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    bothSubsystemsZeroed = m_robotContainer.allZeroed();
    m_robotContainer.setMegaTag2(true);

    m_robotContainer.checkForManualZeroing().cancel();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!hasAutonomousRun) {
      m_robotContainer.zeroSubsystems().schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
