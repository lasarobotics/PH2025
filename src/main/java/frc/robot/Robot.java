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
import frc.robot.subsystems.localization.localizationSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final Servo phlapServo;

  // ✅ Added: reference to localization subsystem
  private final localizationSubsystem m_localization = new localizationSubsystem();

  public Robot() {

    phlapServo = new Servo(3);

    PurpleManager.initialize(
        this,
        Constants.Field.FIELD_LAYOUT,
        Path.of("/media/sda1"),
        BuildConstants.MAVEN_NAME,
        BuildConstants.GIT_SHA,
        BuildConstants.BUILD_DATE,
        false,
        false);

    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();

    RobotController.setBrownoutVoltage(6.75);

    // ✅ Initialize Limelight calibration ranges once here
    m_localization.setCalibrationRanges(
        new double[] {29000, 31000},
        new double[] {12200, 12400},
        new double[] {6200, 6400},
        new double[] {3600, 3800},
        new double[] {1900, 2100});

    // Threads.setCurrentThreadPriority(true, 99);
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    LoopTimer.resetTimer();
    CommandScheduler.getInstance().run();
    LoopTimer.addTimestamp("CommandScheduler");

    // ✅ Feed real-time heading into localization subsystem each loop
    try {
      double headingDeg =
          m_robotContainer.DRIVE_SUBSYSTEM.getPose().getRotation().getDegrees();
      m_localization.setRobotHeadingDeg(headingDeg);
    } catch (Exception e) {
      Logger.recordOutput(
          "Localization/HeadingUpdateError",
          e.getMessage() == null ? "unknown" : e.getMessage());
    }

    Threads.setCurrentThreadPriority(false, 0);
  }

  @Override
  public void disabledInit() {}

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

    // ✅ Optional: log plan outputs for testing
    var plan = m_localization.getPlan();
    Logger.recordOutput("Teleop/AlignRecommendation", plan.recommendation);
    Logger.recordOutput("Teleop/ForwardError_m", plan.forwardErrorMeters);
    Logger.recordOutput("Teleop/StrafeError_m", plan.strafeErrorMeters);
    Logger.recordOutput("Teleop/HeadingError_deg", plan.headingErrorDeg);
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
