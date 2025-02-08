// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Telemetry logger = new Telemetry(Constants.Drive.MAX_SPEED.in(MetersPerSecond));

    private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(), logger);
    private final LiftSubsystem LIFT_SUBSYSTEM = LiftSubsystem.getInstance(LiftSubsystem.initializeHardware());
    private final IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance(IntakeSubsystem.initializeHardware());
    private final EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM = EndEffectorSubsystem.getInstance(EndEffectorSubsystem.initializeHardware());

    private final VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem(VisionSubsystem.initializeHardware());

    private final HeadHoncho HEAD_HONCHO = new HeadHoncho(HeadHoncho.initializeHardware(),
    DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, LIFT_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        HEAD_HONCHO.bindControls(
            () -> joystick.getLeftX(), // drive x
            () -> joystick.getLeftY(), // drive y
            () -> joystick.getRightX(), // drive rotate
            joystick.leftTrigger(), // intake
            joystick.leftBumper(), // regurgitate
            joystick.a(), // L1
            joystick.b(), // L2
            joystick.x(), // L3
            joystick.y(), // L4
            joystick.rightTrigger(), // score
            joystick.rightBumper() // cancel
        );

        joystick.povLeft().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.requestAutoAlign();
        }));

        joystick.povRight().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.cancelAutoAlign();
        }));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
