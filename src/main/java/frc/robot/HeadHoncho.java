package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.EndEffectorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class HeadHoncho extends StateMachine implements AutoCloseable {

  private static TargetLiftStates lastReefState = TargetLiftStates.L4;

  public enum State implements SystemState {

    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    AUTO {
      @Override
      public SystemState nextState() {
            if(!DriverStation.isAutonomous()) return REST;
        return this;
      }
    },
    REST {
      @Override
      public void initialize() {
        // reset the whole robot
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        INTAKE_SUBSYSTEM.stop();
        lastReefState = TargetLiftStates.STOW;
      }

      @Override
      public void execute() {
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
      }

      @Override
      public SystemState nextState() {

        if(s_autoClimb && DriverStation.isFMSAttached() && DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 22.0){
          s_autoClimb = false;
          return MOUNT;
        }

        if (s_intakeButton.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
          return INTAKE;
        }

        if (s_L1Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L1;
        if (s_L2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L2;
        if (s_L3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L3;
        if (s_L4Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L4;

        if(s_climbButtonRising && CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return CLIMB;
        } 
        if(s_climbButtonRising && !CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return MOUNT;
        }

        if(s_algaeL2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L2;
        if(s_algaeL3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L3;

        if(DriverStation.isAutonomous()) return AUTO;

        return this;
      }
    },
    MOUNT {
      @Override
      public void initialize() {
        CLIMB_SUBSYSTEM.mountState();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        if(s_climbButtonRising) return CLIMB;
        if(s_cancelButton.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) {
          return STOW;
        } else if (s_cancelButton.getAsBoolean() && !END_EFFECTOR_SUBSYSTEM.isEmpty()) {
          return TURBO;
        }

        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        CLIMB_SUBSYSTEM.climbState();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        if(s_climbButtonRising) return MOUNT;
        if(s_cancelButton.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) {
          return STOW;
        } else if (s_cancelButton.getAsBoolean() && !END_EFFECTOR_SUBSYSTEM.isEmpty()) {
          return TURBO;
        }
        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        DRIVE_SUBSYSTEM.cancelAutoAlign();
      }

      @Override
      public void execute() {
        if (LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW) && LIFT_SUBSYSTEM.isLiftReady()) {
          INTAKE_SUBSYSTEM.startIntake();
          END_EFFECTOR_SUBSYSTEM.requestIntake();
        }
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
      }

      @Override
      public SystemState nextState() {
        // if (END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return TURBO;
        if (END_EFFECTOR_SUBSYSTEM.isCoralCentered()) {
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

        if(s_climbButtonRising && CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return CLIMB;
        }
        if(s_climbButtonRising && !CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return MOUNT;
        }
        if (s_algaeL2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L2;
        if (s_algaeL3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L3;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        INTAKE_SUBSYSTEM.stop();
        END_EFFECTOR_SUBSYSTEM.requestStop();
      }
    },
    REGURGITATE {
      @Override
      public void initialize() {
        END_EFFECTOR_SUBSYSTEM.requestScoreReverse();
        INTAKE_SUBSYSTEM.startRegurgitate();
      }

      @Override
      public SystemState nextState() {
        return REST;
      }

      @Override
      public void end(boolean interrupted) {
        INTAKE_SUBSYSTEM.stop();
        END_EFFECTOR_SUBSYSTEM.requestStop();
      }
    },
    TURBO {
      @Override
      public void initialize() {
        CLIMB_SUBSYSTEM.setIsMounted(false);
        LIFT_SUBSYSTEM.setState(TargetLiftStates.TURBO);
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        lastReefState = TargetLiftStates.TURBO;
      }

      @Override
      public void execute() {
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
      }

      @Override
      public SystemState nextState() {
        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return STOW;

        if(s_climbButtonRising && CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return CLIMB;
        }
        if(s_climbButtonRising && !CLIMB_SUBSYSTEM.isMounting()) {
          LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
          return MOUNT;
        }
        return this;
      }
    },
    STOW {
      @Override
      public void initialize() {
        CLIMB_SUBSYSTEM.setIsMounted(false);
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        return REST;
      }
    },
    L1 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L1);
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        //DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
        lastReefState = TargetLiftStates.L1;
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE;

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
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L2);
        DRIVE_SUBSYSTEM.requestAutoAlign();
        //DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
        lastReefState = TargetLiftStates.L2;
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
        if (s_forceScoreButton.getAsBoolean() && LIFT_SUBSYSTEM.isLiftReady()) return SCORE;

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
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L3);
        DRIVE_SUBSYSTEM.requestAutoAlign();
       // DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
        lastReefState = TargetLiftStates.L3;
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && DRIVE_SUBSYSTEM.isAligned()) return SCORE_REVERSE;
        if (s_forceScoreButton.getAsBoolean() && LIFT_SUBSYSTEM.isLiftReady()) return SCORE_REVERSE;

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
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
        DRIVE_SUBSYSTEM.requestAutoAlign();
       // DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
        lastReefState = TargetLiftStates.L4;
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && DRIVE_SUBSYSTEM.isAligned()) return SCORE_REVERSE;
        if (s_forceScoreButton.getAsBoolean() && LIFT_SUBSYSTEM.isLiftReady()) return SCORE_REVERSE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return TURBO;

        return this;
      }
    },
    ALGAE_DESCORE_L2 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A1);
        END_EFFECTOR_SUBSYSTEM.requestScore();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        // if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;
        // if (s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        // if (s_algaeL2Button.getAsBoolean()) return ALGAE_DESCORE_L2;
        if (s_algaeL3Button.getAsBoolean()) return ALGAE_DESCORE_L3;

        if (s_cancelButton.getAsBoolean()) return STOW;

        if (s_L1Button.getAsBoolean()) return ALGAE_SCORE_READY;


        return this;
      }
    },
    ALGAE_DESCORE_L3 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A2);
        END_EFFECTOR_SUBSYSTEM.requestScore();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.SLOW_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        // if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;
        // if (s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        if (s_algaeL2Button.getAsBoolean()) return ALGAE_DESCORE_L2;
        // if (s_algaeL3Button.getAsBoolean()) return ALGAE_DESCORE_L3;

        if (s_cancelButton.getAsBoolean()) return STOW;

        if (s_L1Button.getAsBoolean()) return ALGAE_SCORE_READY;

        return this;
      }
    },
    ALGAE_SCORE_READY {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A_SCORE);
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }

      @Override
      public SystemState nextState() {
        if (s_scoreButton.getAsBoolean()) return ALGAE_SCORE;
        if (s_cancelButton.getAsBoolean()) return STOW;

        return this;
      }
    },
    ALGAE_SCORE {
      Timer timer = new Timer();
      @Override
      public void initialize() {
        timer.restart();
        END_EFFECTOR_SUBSYSTEM.requestScoreReverse();
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
      }

      @Override
      public SystemState nextState() {
        if (s_cancelButton.getAsBoolean()) return STOW;
        if (timer.hasElapsed(0.5)) return ALGAE_KICK;
        return this;
      }
    },
    ALGAE_KICK {
      Timer timer = new Timer();
      @Override
      public void initialize() {
        timer.restart();
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A_KICK);
      }

      @Override
      public SystemState nextState() {
        if (s_cancelButton.getAsBoolean()) return STOW;
        if (timer.hasElapsed(0.5)) return STOW;
        return this;
      }
    },
    SCORE {
      @Override
      public void initialize() {
        END_EFFECTOR_SUBSYSTEM.requestScore();
      }

      @Override
      public SystemState nextState() {
        if (s_cancelButton.getAsBoolean()) return STOW;
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return INTAKE;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.TURBO);
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }
    },
    SCORE_REVERSE {
      @Override
      public void initialize() {
        END_EFFECTOR_SUBSYSTEM.requestScoreReverse();
      }

      @Override
      public SystemState nextState() {
        if (s_cancelButton.getAsBoolean()) return STOW;
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return INTAKE;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        DRIVE_SUBSYSTEM.setDriveSpeed(Constants.Drive.FAST_SPEED_SCALAR);
      }
    }
  }

  // Subsystems
  private static DriveSubsystem DRIVE_SUBSYSTEM;
  private static IntakeSubsystem INTAKE_SUBSYSTEM;
  private static LiftSubsystem LIFT_SUBSYSTEM;
  private static EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM;
  private static ClimbSubsystem CLIMB_SUBSYSTEM;

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
  private static Boolean s_autoClimb = true;

  public HeadHoncho(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      LiftSubsystem liftSubsystem,
      EndEffectorSubsystem endEffectorSubsystem,
    ClimbSubsystem climbSubsystem
  ) {
    super(State.REST);

    DRIVE_SUBSYSTEM = driveSubsystem;
    INTAKE_SUBSYSTEM = intakeSubsystem;
    LIFT_SUBSYSTEM = liftSubsystem;
    END_EFFECTOR_SUBSYSTEM = endEffectorSubsystem;
    CLIMB_SUBSYSTEM  = climbSubsystem;
  }

  public void bindControls(
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

    DRIVE_SUBSYSTEM.bindControls(driveRequest, strafeRequest, rotateRequest);
    NamedCommands.registerCommand(Constants.NamedCommands.LIFT_L4_COMMAND_NAME, this.autononomousL4Command());
    NamedCommands.registerCommand(Constants.NamedCommands.LIFT_L4_NO_WAIT_COMMAND_NAME, this.autononomousL4CommandNoWait());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_ALIGN_COMMAND_NAME, this.autononomousAlignCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SCORE_COMMAND_NAME, this.autonomousScoreCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.WAIT_FOR_INTAKE_COMMAND_NAME, this.autonomousWaitForIntakeCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_FIRST_LEFT_CORAL_ALIGN_COMMAND_NAME, this.autoFirstLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SECOND_LEFT_CORAL_ALIGN_COMMAND_NAME, this.autoSecondLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_THIRD_LEFT_CORAL_ALIGN_COMMAND_NAME, this.autoThirdLeftCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_FIRST_RIGHT_CORAL_ALIGN_COMMAND_NAME, this.autoFirstRightCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SECOND_RIGHT_CORAL_ALIGN_COMMAND_NAME, this.autoSecondRightCoralCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_THIRD_RIGHT_CORAL_ALIGN_COMMAND_NAME, this.autoThirdRightCoralCommand());
    NamedCommands.registerCommand("stow climber", this.stowClimberCommand());
  }



  public Command stowClimberCommand() {
    return Commands.startEnd(
      () -> {
        System.out.println("Stow command ran");
        CLIMB_SUBSYSTEM.stow();
      },
      () -> {
        CLIMB_SUBSYSTEM.stopMotor();
        CLIMB_SUBSYSTEM.idleState();
      },
      CLIMB_SUBSYSTEM
    ).until(() -> {return CLIMB_SUBSYSTEM.inStowPosition();});
  }


  /**
   * Tells the robot to move the lift to the L4 state during autonomous
   * @return Command which tells the robot to move the lift to the L4 state during autonomous
   */
  public Command autononomousL4Command() {
    return Commands.startEnd(
            () -> {
              Logger.recordOutput("Auto/Command", Constants.NamedCommands.LIFT_L4_COMMAND_NAME);
              LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
            },
            () -> {},
        this
      )
      .until(() -> {
              return LIFT_SUBSYSTEM.isLiftReady();
            });
  }

  public Command autononomousL4CommandNoWait() {
    return Commands.startEnd(
            () -> {
              Logger.recordOutput("Auto/Command", Constants.NamedCommands.LIFT_L4_COMMAND_NAME);
              LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
            },
            () -> {},
        this
      ).until(() -> true);
  }

  /**
   * Allgns the robot to the reef in auto
   */
  public Command autononomousAlignCommand() {
    return Commands.startEnd(
            () -> {
              Logger.recordOutput("Auto/Command", Constants.NamedCommands.AUTO_ALIGN_COMMAND_NAME);
              DRIVE_SUBSYSTEM.requestAutoAlign();
            },
            () -> {},
        this
      )
      .until(() -> {
              return DRIVE_SUBSYSTEM.isAligned() && LIFT_SUBSYSTEM.isLiftReady();
            });
  }

  /**
   * Tells the robot to score the preload coral during autonomous
   * @return Command that tells the robot to score the preload coral during autononomous
   */
	public Command autonomousScoreCommand() { {
		return
		Commands.startEnd(
              () -> {
        Logger.recordOutput("Auto/Command", Constants.NamedCommands.AUTO_SCORE_COMMAND_NAME);
                END_EFFECTOR_SUBSYSTEM.setState(EndEffectorStates.SCORE_L4);
                DRIVE_SUBSYSTEM.cancelAutoAlign();
              },
              () -> {
                LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
                INTAKE_SUBSYSTEM.startIntake();
                END_EFFECTOR_SUBSYSTEM.requestIntake();
              },
		  this
    )
		.until(() -> {
                return END_EFFECTOR_SUBSYSTEM.isEmpty();
              })
          .withTimeout(0.8);
    }
  }

  /**
   * Sets the intake and end effector subsystems to intake state in autonomous
   * @return Command which sets the intake and end-effector to the intake state in autonomous
   */
  public Command autonomousWaitForIntakeCommand() {
		return Commands.startEnd(() ->
		{}, () -> {
              LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
            },
		this
		)
		.until(() -> {
              return END_EFFECTOR_SUBSYSTEM.forwardBeamBreakBroken();
            });
  }

  /**
   * Auto aligns the robot to a reef in autonomous given an arbitrary pose 
   * @param arbitraryPose arbitrary pose for reef the robot should align to in auto
   */
  private Command autonomousAutoAlignToPoseCommand(Pose2d redPose, Pose2d bluePose) {
    return Commands.startEnd(
    () -> 
      {
        Pose2d arbitraryPose;
        Logger.recordOutput("Autos/autoAlignAlliance", DriverStation.getAlliance().toString());
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
          arbitraryPose = redPose;
        } else {
          arbitraryPose = bluePose;
        }
        Logger.recordOutput("Autos/redPose", redPose);
        Logger.recordOutput("Autos/bluePose", bluePose);
        Logger.recordOutput("Autos/arbitraryPose", arbitraryPose);
        DRIVE_SUBSYSTEM.requestAutoAlign(DRIVE_SUBSYSTEM.findAutoAlignTarget(arbitraryPose));
      },
    () -> {},

    this
    )
    .until(() -> {
      return (DRIVE_SUBSYSTEM.isAligned() && LIFT_SUBSYSTEM.isLiftReady());
      });
    }

 public Command autoFirstLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(12.45, 2.54, new Rotation2d(0.0)); // TODO update this for red alliance
  Pose2d blueAlignPose = new Pose2d(5.03, 5.41, new Rotation2d(0.0));
  Logger.recordOutput("temp/alliance", DriverStation.getAlliance().toString());
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public Command autoSecondLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(13.4, 2.6, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(4.3, 5.5, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public Command autoThirdLeftCoralCommand() {
  Pose2d redAlignPose = new Pose2d(14.3, 2.9, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(3.6, 5.2, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public Command autoFirstRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(12.9, 5.5, new Rotation2d(0.0)); 
  Pose2d blueAlignPose = new Pose2d(4.78, 2.73, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public Command autoSecondRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(13.4, 5.6, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(4.0, 2.5, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

public Command autoThirdRightCoralCommand() {
  Pose2d redAlignPose = new Pose2d(14.1, 5.2, new Rotation2d(0.0));
  Pose2d blueAlignPose = new Pose2d(3.45, 2.8, new Rotation2d(0.0));
  return autonomousAutoAlignToPoseCommand(redAlignPose, blueAlignPose);
}

  @Override
  public void periodic() {
    LoopTimer.addTimestamp(getName() + " Start");
    super.periodic();

    Logger.recordOutput(getName() + "/state", getState().toString());
    if (!s_lastClimbBoolean && s_climbButton.getAsBoolean())
      s_climbButtonRising = true;
    else
      s_climbButtonRising = false;
    s_lastClimbBoolean = s_climbButton.getAsBoolean();

    if (this.getState() == State.SCORE || this.getState() == State.SCORE_REVERSE) {
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
    LoopTimer.addTimestamp(getName() + " End");
  }

  @Override
  public void close() throws Exception {

  }
}
