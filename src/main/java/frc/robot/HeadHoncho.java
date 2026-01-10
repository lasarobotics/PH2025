package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.State;
import frc.robot.lib.StateMachine;
import frc.robot.lib.command.CustomCommands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbStates;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorHardware;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.EndEffectorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class HeadHoncho extends StateMachine {
  private static HeadHoncho s_instance;

  private static TargetLiftStates lastReefState = TargetLiftStates.L4;

  private static BooleanSupplier s_intakeButton;
  private static BooleanSupplier s_forceScoreButton;

  private static BooleanSupplier s_L1Button;
  private static BooleanSupplier s_L2Button;
  private static BooleanSupplier s_L3Button;
  private static BooleanSupplier s_L4Button;

  private static BooleanSupplier s_scoreButton; // force robot to score, regardless of alignment
  private static BooleanSupplier s_cancelButton;

  private static BooleanSupplier s_algaeL2Button;
  private static BooleanSupplier s_algaeL3Button;
  private static BooleanSupplier s_climbButton;
  private static Boolean s_lastClimbBoolean = false;
  private static Boolean s_climbButtonRising = false;

  public static void bindControls(
      DoubleSupplier driveRequest,
      DoubleSupplier strafeRequest,
      DoubleSupplier rotateRequest,
      BooleanSupplier intakeButton,
      BooleanSupplier forceScoreButton,
      BooleanSupplier L1Button,
      BooleanSupplier L2Button,
      BooleanSupplier L3Button,
      BooleanSupplier L4Button,
      BooleanSupplier L2AlgaeButton,
      BooleanSupplier L3AlgaeButton,
      BooleanSupplier scoreButton,
      BooleanSupplier cancelButton,
    BooleanSupplier climbButton
  ) {
    s_intakeButton = intakeButton;
    s_forceScoreButton = forceScoreButton;
    s_L1Button = L1Button;
    s_L2Button = L2Button;
    s_L3Button = L3Button;
    s_L4Button = L4Button;

    s_algaeL2Button = L2AlgaeButton;
    s_algaeL3Button = L3AlgaeButton;

    s_scoreButton = scoreButton;
    s_cancelButton = cancelButton;

    s_climbButton = climbButton;

    DriveSubsystem.getInstance().bindControls(driveRequest, strafeRequest, rotateRequest);
  }

  private HeadHoncho() {
    super(HeadHochoStates.REST);
    NamedCommands.registerCommand(Constants.NamedCommands.LIFT_L4_COMMAND_NAME, autononomousL4Command());
    NamedCommands.registerCommand(Constants.NamedCommands.LIFT_L4_NO_WAIT_COMMAND_NAME, autononomousL4CommandNoWait());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_ALIGN_COMMAND_NAME, autononomousAlignCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SCORE_COMMAND_NAME, autonomousScoreCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.WAIT_FOR_INTAKE_COMMAND_NAME, autonomousWaitForIntakeCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_FIRST_LEFT_CORAL_ALIGN_COMMAND_NAME, autoFirstLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SECOND_LEFT_CORAL_ALIGN_COMMAND_NAME, autoSecondLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_THIRD_LEFT_CORAL_ALIGN_COMMAND_NAME, autoThirdLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_FIRST_RIGHT_CORAL_ALIGN_COMMAND_NAME, autoFirstRightCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SECOND_RIGHT_CORAL_ALIGN_COMMAND_NAME, autoSecondRightCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_THIRD_RIGHT_CORAL_ALIGN_COMMAND_NAME, autoThirdRightCoralCommand());
    NamedCommands.registerCommand("stow climber", stowClimberCommand());
  }

  public static HeadHoncho getInstance() {
    if (HeadHoncho.s_instance == null) {
      HeadHoncho.s_instance = new HeadHoncho();
    }
    return HeadHoncho.s_instance;
  }

  public enum HeadHochoStates implements State {

    NOTHING {
      @Override
      public State nextState() {
        return this;
      }
    },
    AUTO {
      @Override
      public State nextState() {
        if(!DriverStation.isAutonomous()) return REST;
        return this;
      }
    },
    REST {
      @Override
      public void initialize() {
        // reset the whole robot
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.IDLE);
        DriveSubsystem.getInstance().cancelAutoAlign();
        if (!LiftSubsystem.getInstance().isAtState(TargetLiftStates.L4)) {
          LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
        }
        IntakeSubsystem.getInstance().setState(IntakeStates.STOP);
        lastReefState = TargetLiftStates.STOW;
        ClimbSubsystem.getInstance().setState(ClimbStates.IDLE);
      }

      @Override
      public State nextState() {

        if (
          s_intakeButton.getAsBoolean() &&
          EndEffectorHardware.isEmpty() &&
          LiftSubsystem.getInstance().isAtState(TargetLiftStates.STOW)
        ) {
          return INTAKE;
        }

        if (s_L1Button.getAsBoolean() && EndEffectorHardware.isCoralCentered()) return L1;
        if (s_L2Button.getAsBoolean() && EndEffectorHardware.isCoralCentered()) return L2;
        if (s_L3Button.getAsBoolean() && EndEffectorHardware.isCoralCentered()) return L3;
        if (s_L4Button.getAsBoolean() && EndEffectorHardware.isCoralCentered()) return L4;

        if (s_climbButtonRising) return MOUNT;

        if(s_algaeL2Button.getAsBoolean() && EndEffectorHardware.isEmpty()) return ALGAE_DESCORE_L2;
        if(s_algaeL3Button.getAsBoolean() && EndEffectorHardware.isEmpty()) return ALGAE_DESCORE_L3;

        if(DriverStation.isAutonomous()) return AUTO;

        return this;
      }
    },
    MOUNT {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
        ClimbSubsystem.getInstance().setState(ClimbStates.MOUNT);
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public State nextState() {
        if(s_climbButtonRising) return CLIMB;
        if(s_cancelButton.getAsBoolean() && EndEffectorHardware.isEmpty()) {
          ClimbSubsystem.getInstance().setState(ClimbStates.STOW);
          return REST;
        }
        if (s_cancelButton.getAsBoolean() && !EndEffectorHardware.isEmpty()) {
          ClimbSubsystem.getInstance().setState(ClimbStates.STOW);
          return TURBO;
        }

        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
        ClimbSubsystem.getInstance().setState(ClimbStates.CLIMB);
      }

      @Override
      public State nextState() {
        if(s_climbButtonRising) return MOUNT;
        if(s_cancelButton.getAsBoolean() && EndEffectorHardware.isEmpty()) {
          ClimbSubsystem.getInstance().setState(ClimbStates.STOW);
          return REST;
        }
        if (s_cancelButton.getAsBoolean() && !EndEffectorHardware.isEmpty()) {
          ClimbSubsystem.getInstance().setState(ClimbStates.STOW);
          return TURBO;
        }
        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
        DriveSubsystem.getInstance().cancelAutoAlign();
      }

      @Override
      public void execute() {
        if (LiftSubsystem.getInstance().isAtState(TargetLiftStates.STOW) && LiftSubsystem.getInstance().isLiftReady()) {
          IntakeSubsystem.getInstance().setState(IntakeStates.INTAKE);
          EndEffectorSubsystem.getInstance().setState(EndEffectorStates.INTAKE);
        }
      }

      @Override
      public State nextState() {
        if (EndEffectorHardware.isCoralCentered()) {
          switch (lastReefState) {
            case L1:
              return L1;
            case L2:
              return L2;
            case L3:
              return L3;
            case L4:
              return L4;
            default:
              return TURBO;
          }
        }
        if (s_cancelButton.getAsBoolean()) return REST;

        if(s_climbButtonRising) return MOUNT;

        if (s_algaeL2Button.getAsBoolean() && EndEffectorHardware.isEmpty()) return ALGAE_DESCORE_L2;
        if (s_algaeL3Button.getAsBoolean() && EndEffectorHardware.isEmpty()) return ALGAE_DESCORE_L3;

        return this;
      }

      @Override
      public void end(State nextState) {
        IntakeSubsystem.getInstance().setState(IntakeStates.STOP);
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.HOLD);
      }
    },
    TURBO {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.TURBO);
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
        DriveSubsystem.getInstance().cancelAutoAlign();
      }

      @Override
      public State nextState() {
        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (EndEffectorHardware.isEmpty()) return REST;

        if(s_climbButtonRising) return MOUNT;
        return this;
      }
    },
    L1 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L1);
        DriveSubsystem.getInstance().cancelAutoAlign();
        lastReefState = TargetLiftStates.L1;
      }

      @Override
      public State nextState() {
        if (LiftSubsystem.getInstance().isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return TURBO;

        return this;
      }
    },
    L2 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L2);
        DriveSubsystem.getInstance().requestAutoAlign();
        lastReefState = TargetLiftStates.L2;
      }

      @Override
      public State nextState() {
        if (LiftSubsystem.getInstance().isLiftReady() && DriveSubsystem.getInstance().isAligned()) return SCORE;
        if (s_forceScoreButton.getAsBoolean() && LiftSubsystem.getInstance().isLiftReady()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return TURBO;

        return this;
      }
    },
    L3 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L3);
        DriveSubsystem.getInstance().requestAutoAlign();
        lastReefState = TargetLiftStates.L3;
      }

      @Override
      public State nextState() {
        if (LiftSubsystem.getInstance().isLiftReady() && DriveSubsystem.getInstance().isAligned()) return SCORE;
        if (s_forceScoreButton.getAsBoolean() && LiftSubsystem.getInstance().isLiftReady()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return TURBO;

        return this;
      }
    },
    L4 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L4);
        DriveSubsystem.getInstance().requestAutoAlign();
        lastReefState = TargetLiftStates.L4;
      }

      @Override
      public State nextState() {
        if (LiftSubsystem.getInstance().isLiftReady() && DriveSubsystem.getInstance().isAligned()) return SCORE;
        if (s_forceScoreButton.getAsBoolean() && LiftSubsystem.getInstance().isLiftReady()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return TURBO;

        return this;
      }
    },
    SCORE {
      @Override
      public void initialize() {
        if (
          LiftSubsystem.getInstance().isAtState(TargetLiftStates.L1) ||
          LiftSubsystem.getInstance().isAtState(TargetLiftStates.L2)
        ) {
          EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L1_L2);
        }
        else {
          EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L3_L4);
        }
      }

      @Override
      public State nextState() {
        if (s_cancelButton.getAsBoolean()) return REST;
        if (EndEffectorHardware.isEmpty()) return INTAKE;

        return this;
      }

      @Override
      public void end(State nextState) {
        LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.HOLD);
        DriveSubsystem.getInstance().cancelAutoAlign();
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }
    },
    ALGAE_DESCORE_L2 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.A1);
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L1_L2);
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public State nextState() {
        if (s_algaeL3Button.getAsBoolean()) return ALGAE_DESCORE_L3;
        if (s_cancelButton.getAsBoolean()) return REST;
        if (s_L1Button.getAsBoolean()) return ALGAE_SCORE_READY;
        return this;
      }
    },
    ALGAE_DESCORE_L3 {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.A2);
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L1_L2);
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public State nextState() {
        if (s_algaeL2Button.getAsBoolean()) return ALGAE_DESCORE_L2;
        if (s_cancelButton.getAsBoolean()) return REST;
        if (s_L1Button.getAsBoolean()) return ALGAE_SCORE_READY;
        return this;
      }
    },
    ALGAE_SCORE_READY {
      @Override
      public void initialize() {
        LiftSubsystem.getInstance().setState(TargetLiftStates.A_SCORE);
        DriveSubsystem.getInstance().setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }

      @Override
      public State nextState() {
        if (s_scoreButton.getAsBoolean()) return ALGAE_SCORE;
        if (s_cancelButton.getAsBoolean()) return REST;
        return this;
      }
    },
    ALGAE_SCORE {
      Timer timer = new Timer();
      @Override
      public void initialize() {
        timer.restart();
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L3_L4);
        LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
      }

      @Override
      public State nextState() {
        if (s_cancelButton.getAsBoolean()) return REST;
        if (timer.hasElapsed(0.5)) return ALGAE_KICK;
        return this;
      }
    },
    ALGAE_KICK {
      Timer timer = new Timer();
      @Override
      public void initialize() {
        timer.restart();
        LiftSubsystem.getInstance().setState(TargetLiftStates.A_KICK);
      }

      @Override
      public State nextState() {
        if (s_cancelButton.getAsBoolean()) return REST;
        if (timer.hasElapsed(0.5)) return REST;
        return this;
      }
    }
  }

  /**
   * Command to stow the climbeer
   * @return Command which stows the climber
   */
  public static Command stowClimberCommand() {
    return CustomCommands.runOnce(
      () -> {
        ClimbSubsystem.getInstance().setState(ClimbStates.STOW);
      }
    );
  }


  /**
   * Tells the robot to move the lift to the L4 state during autonomous
   * @return Command which tells the robot to move the lift to the L4 state during autonomous
   */
  public static Command autononomousL4Command() {
    return CustomCommands.startEnd(
      () -> {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L4);
      },
      () -> {}
    )
    .until(
      () -> {
        return LiftSubsystem.getInstance().isLiftReady();
      }
    );
  }

  /**
   * Command that goes to L4 without waiting for auto align
   * @return Command that goes to L4 without waiting for auto align
   */
  public static Command autononomousL4CommandNoWait() {
    return CustomCommands.runOnce(
      () -> {
        LiftSubsystem.getInstance().setState(TargetLiftStates.L4);
      }
    );
  }

  /**
   * Allgns the robot to the reef in auto
   */
  public static Command autononomousAlignCommand() {
    return CustomCommands.runOnce(
      () -> {
        DriveSubsystem.getInstance().requestAutoAlign();
      }
    )
    .until(
      () -> {
        return DriveSubsystem.getInstance().isAligned() && LiftSubsystem.getInstance().isLiftReady();
      }
    );
  }

  /**
   * Tells the robot to score the preload coral during autonomous
   * @return Command that tells the robot to score the preload coral during autononomous
   */
	public static Command autonomousScoreCommand() {
		return CustomCommands.runOnce(
      () -> {
        EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L3_L4);
        DriveSubsystem.getInstance().cancelAutoAlign();
      }
    )
		.until(
      () -> {
        return EndEffectorHardware.isEmpty();
      }
    )
    .withTimeout(0.8)
    .andThen(
      CustomCommands.startEnd(
        () -> {
          LiftSubsystem.getInstance().setState(TargetLiftStates.PANIC);
          EndEffectorSubsystem.getInstance().setState(EndEffectorStates.SCORE_L3_L4);
        },
        () -> {
          LiftSubsystem.getInstance().setState(TargetLiftStates.STOW);
          IntakeSubsystem.getInstance().setState(IntakeStates.INTAKE);
          EndEffectorSubsystem.getInstance().setState(EndEffectorStates.INTAKE);
        }
      )
      .until(
        () -> {
          return EndEffectorHardware.isEmpty();
        }
      )
      .withTimeout(0.5)
    );
  }




  /**
   * Sets the intake and end effector subsystems to intake state in autonomous
   * @return Command which sets the intake and end-effector to the intake state in autonomous
   */
  public static Command autonomousWaitForIntakeCommand() {
		return Commands.waitUntil(
      () -> {
        return EndEffectorHardware.forwardBeamBreakBroken();
      }
    ).andThen(
      CustomCommands.runOnce(
        () -> {
          LiftSubsystem.getInstance().setState(TargetLiftStates.L4);
        }
      )
    );
  }

  /**
   * Auto aligns the robot to a reef in autonomous given an arbitrary pose 
   * @param arbitraryPose arbitrary pose for reef the robot should align to in auto
   */
  public static Command autonomousAutoAlignToPoseCommand(Pose2d redPose, Pose2d bluePose) {
    return CustomCommands.runOnce(
      () -> {
        Pose2d arbitraryPose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
          arbitraryPose = redPose;
        } else {
          arbitraryPose = bluePose;
        }
        DriveSubsystem.getInstance().requestAutoAlign(DriveSubsystem.getInstance().findAutoAlignTarget(arbitraryPose));
      }
    )
    .until(
      () -> {
        return (DriveSubsystem.getInstance().isAligned() && LiftSubsystem.getInstance().isLiftReady());
      }
    );
  }

 public static Command autoFirstLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(12.45, 2.54, new Rotation2d(0.0)); // TODO update this for red alliance
  Pose2d blueAlignPose = new Pose2d(5.03, 5.41, new Rotation2d(0.0));
  Logger.recordOutput("temp/alliance", DriverStation.getAlliance().toString());
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public static Command autoSecondLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(13.4, 2.6, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(4.3, 5.5, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public static Command autoThirdLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(14.3, 2.9, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(3.6, 5.2, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public static Command autoFirstRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(12.9, 5.5, new Rotation2d(0.0)); 
  Pose2d blueAlignPose = new Pose2d(4.78, 2.73, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public static Command autoSecondRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(13.4, 5.6, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(4.0, 2.5, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public static Command autoThirdRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(14.1, 5.2, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(3.45, 2.8, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

  @Override
  public void periodic() {
    super.periodic();
    if (!s_lastClimbBoolean && s_climbButton.getAsBoolean())
      s_climbButtonRising = true;
    else
      s_climbButtonRising = false;
    s_lastClimbBoolean = s_climbButton.getAsBoolean();

    if (s_L4Button.getAsBoolean()) {
      lastReefState = TargetLiftStates.L4;
    }
    if (s_L3Button.getAsBoolean()) {
      lastReefState = TargetLiftStates.L3;
    }
    if (s_L2Button.getAsBoolean()) {
      lastReefState = TargetLiftStates.L2;
    }
    if (s_L1Button.getAsBoolean()) {
      lastReefState = TargetLiftStates.L1;
    }

    if (this.getState() == HeadHochoStates.SCORE) {
      RobotContainer.setViolet();
    }
    else if (lastReefState == TargetLiftStates.L4) {
      RobotContainer.setGreen();
    }
    else if (lastReefState == TargetLiftStates.L3) {
      RobotContainer.setBlue();
    }
    else if (lastReefState == TargetLiftStates.L2) {
      RobotContainer.setYellow();
    }
    else if (lastReefState == TargetLiftStates.L1) {
      RobotContainer.setRed();
    }
    else {
      RobotContainer.setRainbow();
    }

    Logger.recordOutput(getName() + "/buttons/L1", s_L1Button);
    Logger.recordOutput(getName() + "/buttons/L2", s_L2Button);
    Logger.recordOutput(getName() + "/buttons/L3", s_L3Button);
    Logger.recordOutput(getName() + "/buttons/L4", s_L4Button);

    Logger.recordOutput(getName() + "/buttons/score", s_scoreButton);
    Logger.recordOutput(getName() + "/buttons/cancel", s_cancelButton);

    Logger.recordOutput(getName() + "/buttons/algaeL2", s_algaeL2Button);
    Logger.recordOutput(getName() + "/buttons/algaeL3", s_algaeL3Button);

    Logger.recordOutput(getName() + "/buttons/intake", s_intakeButton);
  }
}
