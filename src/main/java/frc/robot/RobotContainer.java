// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Telemetry logger = new Telemetry(Constants.Drive.MAX_SPEED.in(MetersPerSecond));

  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(), logger);
  private final LiftSubsystem LIFT_SUBSYSTEM = LiftSubsystem.getInstance(LiftSubsystem.initializeHardware());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    DRIVE_SUBSYSTEM.bindControls(() -> -joystick.getLeftY(), () -> -joystick.getLeftX(), () -> -joystick.getRightX());
    joystick.a().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kForward));
    joystick.b().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse));
    joystick.x().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
