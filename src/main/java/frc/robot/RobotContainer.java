// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Telemetry logger = new Telemetry(Constants.Drive.MAX_SPEED.in(MetersPerSecond));

  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(), logger);
  private final LiftSubsystem LIFT_SUBSYSTEM = LiftSubsystem.getInstance(LiftSubsystem.initializeHardware());

    private final VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem(VisionSubsystem.initializeHardware());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        DRIVE_SUBSYSTEM.bindControls(() -> -joystick.getLeftY(), () -> -joystick.getLeftX(),
                () -> -joystick.getRightX());


        joystick.a().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.requestAutoAlign(new Pose2d(10.0, 10.0, new Rotation2d(3.1415/2)));
        }));
        joystick.b().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.cancelAutoAlign();;
        }));
        joystick.b().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.requestAutoAlign(null);
        }));

        joystick.x().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.requestAutoAlign();
        }));
        joystick.b().onTrue(Commands.runOnce(() -> {
            DRIVE_SUBSYSTEM.cancelAutoAlign();;
        }));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y. 
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
        // drivetrain.seedFieldCentric()));
      joystick.a().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kForward));
    joystick.b().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse));
    joystick.x().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse));
  }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
