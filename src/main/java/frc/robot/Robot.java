// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

  @Logged
  public class PDHValues {
    @NotLogged
    PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
    Voltage voltage = Volts.of(PDH.getVoltage());
    Current PORT0 = Amps.of(PDH.getCurrent(0));
    Current PORT1 = Amps.of(PDH.getCurrent(1));
    Current PORT2 = Amps.of(PDH.getCurrent(2));
    Current PORT3 = Amps.of(PDH.getCurrent(3));
    Current PORT4 = Amps.of(PDH.getCurrent(4));
    Current PORT5 = Amps.of(PDH.getCurrent(5));
    Current PORT6 = Amps.of(PDH.getCurrent(6));
    Current PORT7 = Amps.of(PDH.getCurrent(7));
    Current PORT8 = Amps.of(PDH.getCurrent(8));
    Current PORT9 = Amps.of(PDH.getCurrent(9));
    Current PORT10 = Amps.of(PDH.getCurrent(10));
    Current PORT11 = Amps.of(PDH.getCurrent(11));
    Current PORT12 = Amps.of(PDH.getCurrent(12));
    Current PORT13 = Amps.of(PDH.getCurrent(13));
    Current PORT14 = Amps.of(PDH.getCurrent(14));
    Current PORT15 = Amps.of(PDH.getCurrent(15));
    Current PORT16 = Amps.of(PDH.getCurrent(16));
    Current PORT17 = Amps.of(PDH.getCurrent(17));
    Current PORT18 = Amps.of(PDH.getCurrent(18));
    Current PORT19 = Amps.of(PDH.getCurrent(19));
    Current PORT20 = Amps.of(PDH.getCurrent(20));
    Current PORT21 = Amps.of(PDH.getCurrent(21));
    Current PORT22 = Amps.of(PDH.getCurrent(22));
    Current PORT23 = Amps.of(PDH.getCurrent(23));

  }

  PDHValues pdhValues = new PDHValues();
}
