// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final AutoHoncho AUTO_HONCHO = new AutoHoncho(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, LIFT_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM);
  private static SendableChooser<Command> m_autoModeChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    m_autoModeChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Mode", m_autoModeChooser);

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
     PRIMARY_CONTROLLER.y(), // L3
     PRIMARY_CONTROLLER.rightTrigger(), // L4
     PRIMARY_CONTROLLER.povDown(), //L2 Algae Descore
     PRIMARY_CONTROLLER.povUp(), //L3 Algae Descore
     PRIMARY_CONTROLLER.rightBumper(), // score
     PRIMARY_CONTROLLER.x() // cancel
    );

    // PRIMARY_CONTROLLER.x().whileTrue(DRIVE_SUBSYSTEM.m_sysIdRoutineToApply.dynamic(SysIdRoutine.Direction.kForward));

    // PRIMARY_CONTROLLER.y().onTrue(Commands.runOnce(() -> {
    //   LIFT_SUBSYSTEM.setState(TargetLiftStates.L1);
    // }));

    // PRIMARY_CONTROLLER.b().onTrue(Commands.runOnce(() -> {
    //   LIFT_SUBSYSTEM.setState(TargetLiftStates.L2);
    // }));

    // PRIMARY_CONTROLLER.a().onTrue(Commands.runOnce(() -> {
    //   LIFT_SUBSYSTEM.setState(TargetLiftStates.L3);
    // }));

    // PRIMARY_CONTROLLER.rightTrigger().onTrue(Commands.runOnce(() -> {
    //   LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
    // }));
      

    // PRIMARY_CONTROLLER.povLeft().onTrue(Commands.runOnce(() -> {
    //   DRIVE_SUBSYSTEM.requestAutoAlign();
    // }));
    // PRIMARY_CONTROLLER.povRight().onTrue(Commands.runOnce(() -> {
    //   DRIVE_SUBSYSTEM.cancelAutoAlign();
    // }));

    // PRIMARY_CONTROLLER.a().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kForward));
    // PRIMARY_CONTROLLER.b().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse));
    // PRIMARY_CONTROLLER.x().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward));
    // PRIMARY_CONTROLLER.y().whileTrue(LIFT_SUBSYSTEM.getElevatorSysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse));


    // PRIMARY_CONTROLLER.a().onTrue(Commands.runOnce(() -> {
    //   System.out.println("a");
    //   INTAKE_SUBSYSTEM.startIntake();
    // }));
    // PRIMARY_CONTROLLER.b().onTrue(Commands.runOnce(() -> {
    //   INTAKE_SUBSYSTEM.startRegurgitate();
    // }));
    // PRIMARY_CONTROLLER.x().onTrue(Commands.runOnce(() -> {
    //   INTAKE_SUBSYSTEM.stop();
    // }));

    PRIMARY_CONTROLLER.povLeft().onTrue(
      Commands.runOnce(() -> {
        DRIVE_SUBSYSTEM.resetPose();
      })
    );
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_autoModeChooser.setDefaultOption("Do nothing", Commands.none());
    m_autoModeChooser.setDefaultOption(Constants.AutoNames.TEST_AUTO_NAME.getFirst(), new PathPlannerAuto(Constants.AutoNames.TEST_AUTO_NAME.getSecond()));
  }

  //Register named commands for pathplanner
  

  public Command getAutonomousCommand() {
    return m_autoModeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_autoModeChooser);
  }
}
