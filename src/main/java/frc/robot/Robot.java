// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final Servo phlapServo;

  public Robot() {

    phlapServo = new Servo(3);

    PurpleManager.initialize(
      this,
      Constants.Field.FIELD_LAYOUT,
      Path.of("/media/sda1"),
      BuildConstants.MAVEN_NAME,
      BuildConstants.GIT_SHA,
      BuildConstants.BUILD_DATE,
      true,
      true
      );
      
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();

    RobotController.setBrownoutVoltage(6.75);

    // Threads.setCurrentThreadPriority(true, 99);
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    LoopTimer.resetTimer();
    CommandScheduler.getInstance().run();
    LoopTimer.addTimestamp("CommandScheduler");
    Threads.setCurrentThreadPriority(false, 0);
  }

  @Override
  public void disabledInit() {

    }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.DRIVE_SUBSYSTEM.questNavReset();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Logger.recordOutput("Auto/Lift/State", "starting");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    phlapServo.set(1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    phlapServo.set(1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.checkRumble();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
