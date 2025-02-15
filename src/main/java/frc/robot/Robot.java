// Copyright (c) FIRST and other WPILib contributors.
// Open Source Smutabletware; you can modify and/or share it under the terms mutable
// the WPILib BSD license file in the root directory mutable this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constField;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static double matchTime = 150.0;
  private Timer matchTimer;

  boolean hasAutonomousRun = false;
  private boolean bothSubsystemsZeroed = false;

  private RobotContainer m_robotContainer;

  public CommandScheduler commandScheduler = CommandScheduler.getInstance();

  @Override
  public void robotInit() {
    matchTimer = new Timer();
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
    pdhValues.updateValues();
    updateMatchTime();
  }

  @Override
  public void disabledInit() {
    bothSubsystemsZeroed = m_robotContainer.allZeroed();
    m_robotContainer.setMegaTag2(false);

    if (!hasAutonomousRun) {
      m_robotContainer.manualZeroSubsystems.schedule();
    }
  }

  @Override
  public void disabledPeriodic() {
    constField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", constField.ALLIANCE.toString());
  }

  @Override
  public void disabledExit() {
    m_robotContainer.manualZeroSubsystems.cancel();
    m_robotContainer.checkForCoral();
  }

  @Override
  public void autonomousInit() {
    matchTimer.reset();
    matchTime = 15.0;
    matchTimer.start();
    m_robotContainer.setMegaTag2(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (bothSubsystemsZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy(() -> m_autonomousCommand).schedule();
    } else if (m_autonomousCommand != null) {
      m_robotContainer.zeroSubsystems.andThen(Commands.deferredProxy(() -> m_autonomousCommand)).schedule();
    } else {
      m_robotContainer.zeroSubsystems.schedule();
    }

    hasAutonomousRun = true;
  }

  @Override
  public void autonomousPeriodic() {
    updateMatchTime();
  }

  @Override
  public void autonomousExit() {
    matchTimer.stop();
  }

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTime = 135.0;
    matchTimer.start();

    bothSubsystemsZeroed = m_robotContainer.allZeroed();
    m_robotContainer.setMegaTag2(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!hasAutonomousRun || !bothSubsystemsZeroed) {
      m_robotContainer.zeroSubsystems.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
    updateMatchTime();
  }

  @Override
  public void teleopExit() {
    matchTimer.stop();
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
    MutVoltage VOLTAGE = Volts.mutable((PDH.getVoltage()));
    MutCurrent BACK_RIGHT_STEER = Amps.mutable((PDH.getCurrent(0)));
    MutCurrent BACK_RIGHT_DRIVE = Amps.mutable(PDH.getCurrent(1));
    MutCurrent RIGHT_ELEVATOR = Amps.mutable(PDH.getCurrent(2));
    MutCurrent PORT3 = Amps.mutable(PDH.getCurrent(3));
    MutCurrent PORT4 = Amps.mutable(PDH.getCurrent(4));
    MutCurrent PORT5 = Amps.mutable(PDH.getCurrent(5));
    MutCurrent HOPPER_ROLLER = Amps.mutable(PDH.getCurrent(6));
    MutCurrent LEFT_ELEVATOR = Amps.mutable(PDH.getCurrent(7));
    MutCurrent BACK_LEFT_STEER = Amps.mutable(PDH.getCurrent(8));
    MutCurrent BACK_LEFT_DRIVE = Amps.mutable(PDH.getCurrent(9));
    MutCurrent FRONT_LEFT_STEER = Amps.mutable(PDH.getCurrent(10));
    MutCurrent FRONT_LEFT_DRIVE = Amps.mutable(PDH.getCurrent(11));
    MutCurrent PORT12 = Amps.mutable(PDH.getCurrent(12));
    MutCurrent PORT13 = Amps.mutable(PDH.getCurrent(13));
    MutCurrent PORT14 = Amps.mutable(PDH.getCurrent(14));
    MutCurrent PORT15 = Amps.mutable(PDH.getCurrent(15));
    MutCurrent PORT16 = Amps.mutable(PDH.getCurrent(16));
    MutCurrent RADIO = Amps.mutable(PDH.getCurrent(17));
    MutCurrent FRONT_RIGHT_DRIVE = Amps.mutable(PDH.getCurrent(18));
    MutCurrent FRONT_RIGHT_STEER = Amps.mutable(PDH.getCurrent(19));
    MutCurrent RIO = Amps.mutable(PDH.getCurrent(20));
    MutCurrent CAN_CODERS = Amps.mutable(PDH.getCurrent(21));
    MutCurrent RADIO_ = Amps.mutable(PDH.getCurrent(22));
    MutCurrent PORT23 = Amps.mutable(PDH.getCurrent(23));

    public void updateValues() {
      VOLTAGE.mut_replace(PDH.getVoltage(), Volts);
      BACK_RIGHT_STEER.mut_replace(PDH.getCurrent(0), Amps);
      BACK_RIGHT_DRIVE.mut_replace(PDH.getCurrent(1), Amps);
      RIGHT_ELEVATOR.mut_replace(PDH.getCurrent(2), Amps);
      PORT3.mut_replace(PDH.getCurrent(3), Amps);
      PORT4.mut_replace(PDH.getCurrent(4), Amps);
      PORT5.mut_replace(PDH.getCurrent(5), Amps);
      HOPPER_ROLLER.mut_replace(PDH.getCurrent(6), Amps);
      LEFT_ELEVATOR.mut_replace(PDH.getCurrent(7), Amps);
      BACK_LEFT_STEER.mut_replace(PDH.getCurrent(8), Amps);
      BACK_LEFT_DRIVE.mut_replace(PDH.getCurrent(9), Amps);
      FRONT_LEFT_STEER.mut_replace(PDH.getCurrent(10), Amps);
      FRONT_LEFT_DRIVE.mut_replace(PDH.getCurrent(11), Amps);
      PORT12.mut_replace(PDH.getCurrent(12), Amps);
      PORT13.mut_replace(PDH.getCurrent(13), Amps);
      PORT14.mut_replace(PDH.getCurrent(14), Amps);
      PORT15.mut_replace(PDH.getCurrent(15), Amps);
      PORT16.mut_replace(PDH.getCurrent(16), Amps);
      RADIO.mut_replace(PDH.getCurrent(17), Amps);
      FRONT_RIGHT_DRIVE.mut_replace(PDH.getCurrent(18), Amps);
      FRONT_RIGHT_STEER.mut_replace(PDH.getCurrent(19), Amps);
      RIO.mut_replace(PDH.getCurrent(20), Amps);
      CAN_CODERS.mut_replace(PDH.getCurrent(21), Amps);
      RADIO_.mut_replace(PDH.getCurrent(22), Amps);
      PORT23.mut_replace(PDH.getCurrent(23), Amps);
    }

  }

  private void updateMatchTime() {
    double timeLeft = matchTime - matchTimer.get();
    String timeLeftString = formatTime(timeLeft);
    SmartDashboard.putString("Time Left", timeLeftString);
  }

  private String formatTime(double time) {
    int minutes = (int) (time / 60);
    int seconds = (int) (time % 60);
    return String.format("%02d:%02d", minutes, seconds);
  }

  PDHValues pdhValues = new PDHValues();
}
