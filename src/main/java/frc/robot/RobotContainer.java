// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class RobotContainer {
  private final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(0);
  private final Telemetry LOGGER = new Telemetry(Constants.Drive.MAX_SPEED.in(MetersPerSecond));

  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(), LOGGER);
  private final LiftSubsystem LIFT_SUBSYSTEM = LiftSubsystem.getInstance(LiftSubsystem.initializeHardware());
  private final IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance(IntakeSubsystem.initializeHardware());
  private final EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM = EndEffectorSubsystem.getInstance(EndEffectorSubsystem.initializeHardware());
  private final HeadHoncho HEAD_HONCHO = new HeadHoncho(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, LIFT_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    HEAD_HONCHO.bindControls(
      () -> PRIMARY_CONTROLLER.getLeftX(), // drive x
      () -> PRIMARY_CONTROLLER.getLeftY(), // drive y
      () -> PRIMARY_CONTROLLER.getRightX(), // drive rotate
      PRIMARY_CONTROLLER.leftTrigger(), // intake
      PRIMARY_CONTROLLER.leftBumper(), // regurgitate
      PRIMARY_CONTROLLER.a(), // L1
      PRIMARY_CONTROLLER.b(), // L2
      PRIMARY_CONTROLLER.x(), // L3
      PRIMARY_CONTROLLER.y(), // L4
      PRIMARY_CONTROLLER.rightTrigger(), // score
      PRIMARY_CONTROLLER.rightBumper() // cancel
    );

    PRIMARY_CONTROLLER.povLeft().onTrue(Commands.runOnce(() -> {
      DRIVE_SUBSYSTEM.requestAutoAlign();
    }));

    PRIMARY_CONTROLLER.povRight().onTrue(Commands.runOnce(() -> {
      DRIVE_SUBSYSTEM.cancelAutoAlign();
    }));

    PRIMARY_CONTROLLER.a().onTrue(Commands.runOnce(() -> {
      System.out.println("a");
      INTAKE_SUBSYSTEM.startIntake();
    }));
    PRIMARY_CONTROLLER.b().onTrue(Commands.runOnce(() -> {
      INTAKE_SUBSYSTEM.startRegurgitate();
    }));
    PRIMARY_CONTROLLER.x().onTrue(Commands.runOnce(() -> {
      INTAKE_SUBSYSTEM.stop();
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
