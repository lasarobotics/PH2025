// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class RobotContainer {
  private final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(0);
  // private final CommandXboxController SECONDARY_CONTROLLER = new CommandXboxController(1);
  private static final Telemetry LOGGER = new Telemetry(Constants.Drive.MAX_SPEED.in(MetersPerSecond));

  public static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(), LOGGER);
  private final LiftSubsystem LIFT_SUBSYSTEM = LiftSubsystem.getInstance(LiftSubsystem.initializeHardware());
  private final IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance(IntakeSubsystem.initializeHardware());
  private final EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM = EndEffectorSubsystem.getInstance(EndEffectorSubsystem.initializeHardware(), LIFT_SUBSYSTEM);
  private final ClimbSubsystem CLIMB_SUBSYSTEM = ClimbSubsystem.getInstance(ClimbSubsystem.initializeHardware());
  private final static Led LED_SUBSYSTEM = Led.getInstance(Led.initializHardware());
  private final HeadHoncho HEAD_HONCHO = new HeadHoncho(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, LIFT_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM, CLIMB_SUBSYSTEM);
  // private final AutoHoncho AUTO_HONCHO = new AutoHoncho(DRIVE_SUBSYSTEM, INTAKE_SUBSYSTEM, LIFT_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM);
  private static SendableChooser<Command> m_autoModeChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    m_autoModeChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putBoolean("Is Ready", DRIVE_SUBSYSTEM.isAligned() && LIFT_SUBSYSTEM.isLiftReady());
    SmartDashboard.putData("Auto Mode", m_autoModeChooser);
    DRIVE_SUBSYSTEM.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    LED_SUBSYSTEM.setViolet();
  }

  private void configureBindings() {
    HEAD_HONCHO.bindControls(
     () -> PRIMARY_CONTROLLER.getLeftX(), // drive x
     () -> PRIMARY_CONTROLLER.getLeftY(), // drive y
     () -> PRIMARY_CONTROLLER.getRightX(), // drive rotate
     PRIMARY_CONTROLLER.leftTrigger(), // intake
     PRIMARY_CONTROLLER.leftBumper(), // force score 
     PRIMARY_CONTROLLER.a(), // L1
     PRIMARY_CONTROLLER.b(), // L2
     PRIMARY_CONTROLLER.y(), // L3
     PRIMARY_CONTROLLER.rightBumper(), // L4
     PRIMARY_CONTROLLER.povDown(), //L2 Algae Descore
     PRIMARY_CONTROLLER.povUp(), //L3 Algae Descore
     PRIMARY_CONTROLLER.rightTrigger(), // score
     PRIMARY_CONTROLLER.x(), // cancel
     PRIMARY_CONTROLLER.rightStick() //climb
    );

    PRIMARY_CONTROLLER.povLeft().onTrue(Commands.runOnce(() -> {
      DRIVE_SUBSYSTEM.requestAutoAlign();
    }));
    PRIMARY_CONTROLLER.povRight().onTrue(Commands.runOnce(() -> {
      DRIVE_SUBSYSTEM.cancelAutoAlign();
    }));

    // SECONDARY_CONTROLLER.povUp().whileTrue(CLIMB_SUBSYSTEM.raiseClimberCommand());
    // SECONDARY_CONTROLLER.povDown().whileTrue(CLIMB_SUBSYSTEM.lowerClimberCommand());

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

    PRIMARY_CONTROLLER.back().onTrue(
      Commands.runOnce(() -> {
        DRIVE_SUBSYSTEM.resetPose();
      })
    );
  }

  /**
   * Rumble the controller if the drivetrain is aligned and the lift is ready
   */
  public void checkRumble() {
    if (DRIVE_SUBSYSTEM.isAligned() && LIFT_SUBSYSTEM.isLiftReady())
      PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kRightRumble, 1.0);
    else
      PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kRightRumble, 0.0);
  }

  /**
   * setter methods for led colors
   */
  public static void setViolet() {
    LED_SUBSYSTEM.setViolet();
  }

  public static void setRed() {
    LED_SUBSYSTEM.setRed();
  }

  public static void setYellow() {
    LED_SUBSYSTEM.setYellow();
  }

  public static void setBlue() {
    LED_SUBSYSTEM.setBlue();
  }

  public static void setWhite() {
    LED_SUBSYSTEM.setWhite();
  }

  public static void setGreen() {
    LED_SUBSYSTEM.setGreen();
  }

  public static void setRainbow() {
    LED_SUBSYSTEM.setRainbow();
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_autoModeChooser.setDefaultOption("Do nothing", Commands.none());
    m_autoModeChooser.setDefaultOption(Constants.AutoNames.TEST_AUTO_NAME.getFirst(), new PathPlannerAuto(Constants.AutoNames.TEST_AUTO_NAME.getSecond()));
    m_autoModeChooser.setDefaultOption(Constants.AutoNames.PRELOAD_1A_AUTO_NAME.getFirst(), new PathPlannerAuto(Constants.AutoNames.PRELOAD_1A_AUTO_NAME.getSecond()));
    m_autoModeChooser.setDefaultOption(Constants.AutoNames.THREE_LEFT_CORAL_AUTO_NAME.getFirst(), new PathPlannerAuto(Constants.AutoNames.THREE_LEFT_CORAL_AUTO_NAME.getSecond()));
    m_autoModeChooser.setDefaultOption(Constants.AutoNames.THREE_RIGHT_CORAL_AUTO_NAME.getFirst(), new PathPlannerAuto(Constants.AutoNames.THREE_RIGHT_CORAL_AUTO_NAME.getSecond()));
    m_autoModeChooser.setDefaultOption("Stow climber", Commands.startEnd(() -> {
      CLIMB_SUBSYSTEM.stow();
    }, () -> {

    }, CLIMB_SUBSYSTEM));
    PathfindingCommand.warmupCommand().schedule();
  }

  public void disabledPeriodic() {
    DRIVE_SUBSYSTEM.questNavReset();
  }

  public Command getAutonomousCommand() {
    Logger.recordOutput("Autos/selectedAuto", m_autoModeChooser.getSelected().getName());
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
