package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.EndEffectorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class HeadHoncho extends StateMachine implements AutoCloseable {
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
      }

      @Override
      public SystemState nextState() {
        if (s_intakeButton.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
          return INTAKE;
        }

        if (s_regurgitateButton.getAsBoolean() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
          return REGURGITATE;
        }

        if (s_L1Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L1;
        if (s_L2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L2;
        if (s_L3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L3;
        if (s_L4Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L4;

        if(s_algaeL2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L2;
        if(s_algaeL3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return ALGAE_DESCORE_L3;

        if(DriverStation.isAutonomous()) return AUTO;

        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        DRIVE_SUBSYSTEM.cancelAutoAlign();
        INTAKE_SUBSYSTEM.startIntake();
        END_EFFECTOR_SUBSYSTEM.requestIntake();
      }

      @Override
      public SystemState nextState() {
        if (END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return REST;
        if (s_cancelButton.getAsBoolean()) return REST;

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
        if (s_cancelButton.getAsBoolean() && (INTAKE_SUBSYSTEM.coralFullyInIntake() || END_EFFECTOR_SUBSYSTEM.isCoralCentered())) return REST;
        // TODO algae

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        INTAKE_SUBSYSTEM.stop();
        END_EFFECTOR_SUBSYSTEM.requestStop();
      }
    },
    STOW {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
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
        DRIVE_SUBSYSTEM.requestAutoAlign();
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return STOW;

        return this;
      }
    },
    L2 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L2);
        DRIVE_SUBSYSTEM.requestAutoAlign();
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return STOW;

        return this;
      }
    },
    L3 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L3);
        DRIVE_SUBSYSTEM.requestAutoAlign();
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return STOW;

        return this;
      }
    },
    L4 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
        DRIVE_SUBSYSTEM.requestAutoAlign();
      }

      @Override
      public SystemState nextState() {
        if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

        if (s_cancelButton.getAsBoolean()) return STOW;

        return this;
      }
    },
    ALGAE_DESCORE_L2 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A1);
        END_EFFECTOR_SUBSYSTEM.requestScore();
      }

      @Override
      public SystemState nextState() {
        // if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;
        // if (s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        // if (s_algaeL2Button.getAsBoolean()) return ALGAE_DESCORE_L2;
        if (s_algaeL3Button.getAsBoolean()) return ALGAE_DESCORE_L3;

        if (s_cancelButton.getAsBoolean()) return STOW;


        return this;
      }
    },
    ALGAE_DESCORE_L3 {
      @Override
      public void initialize() {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.A2);
        END_EFFECTOR_SUBSYSTEM.requestScore();
      }

      @Override
      public SystemState nextState() {
        // if (LIFT_SUBSYSTEM.isLiftReady() && s_scoreButton.getAsBoolean()) return SCORE_REVERSE;
        // if (s_scoreButton.getAsBoolean()) return SCORE_REVERSE;

        if (s_algaeL2Button.getAsBoolean()) return ALGAE_DESCORE_L2;
        // if (s_algaeL3Button.getAsBoolean()) return ALGAE_DESCORE_L3;

        if (s_cancelButton.getAsBoolean()) return STOW;

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
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return INTAKE;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
      }
    },
    SCORE_REVERSE {
      @Override
      public void initialize() {
        END_EFFECTOR_SUBSYSTEM.requestScoreReverse();
      }

      @Override
      public SystemState nextState() {
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return INTAKE;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
      }
    }
  }

  // Subsystems
  private static DriveSubsystem DRIVE_SUBSYSTEM;
  private static IntakeSubsystem INTAKE_SUBSYSTEM;
  private static LiftSubsystem LIFT_SUBSYSTEM;
  private static EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM;

  private static BooleanSupplier s_intakeButton;
  private static BooleanSupplier s_regurgitateButton;

  private static BooleanSupplier s_L1Button;
  private static BooleanSupplier s_L2Button;
  private static BooleanSupplier s_L3Button;
  private static BooleanSupplier s_L4Button;

  private static BooleanSupplier s_scoreButton; // force robot to score, regardless of alignment
  private static BooleanSupplier s_cancelButton;

  private static BooleanSupplier s_algaeL2Button;
  private static BooleanSupplier s_algaeL3Button;

  public HeadHoncho(
    DriveSubsystem driveSubsystem,
    IntakeSubsystem intakeSubsystem,
    LiftSubsystem liftSubsystem,
    EndEffectorSubsystem endEffectorSubsystem
  ) {
    super(State.REST);

    DRIVE_SUBSYSTEM = driveSubsystem;
    INTAKE_SUBSYSTEM = intakeSubsystem;
    LIFT_SUBSYSTEM = liftSubsystem;
    END_EFFECTOR_SUBSYSTEM = endEffectorSubsystem;
  }

  public void bindControls(
    DoubleSupplier driveRequest,
    DoubleSupplier strafeRequest,
    DoubleSupplier rotateRequest,
    BooleanSupplier intakeButton,
    BooleanSupplier regurgitateButton,
    BooleanSupplier L1Button,
    BooleanSupplier L2Button,
    BooleanSupplier L3Button,
    BooleanSupplier L4Button,
    BooleanSupplier L2AlgaeButton,
    BooleanSupplier L3AlgaeButton,
    BooleanSupplier scoreButton,
    BooleanSupplier cancelButton
  ) {
    s_intakeButton = intakeButton;
    s_regurgitateButton = regurgitateButton;
    s_L1Button = L1Button;
    s_L2Button = L2Button;
    s_L3Button = L3Button;
    s_L4Button = L4Button;

    s_algaeL2Button = L2AlgaeButton;
    s_algaeL3Button = L3AlgaeButton;

    s_scoreButton = scoreButton;
    s_cancelButton = cancelButton;

    DRIVE_SUBSYSTEM.bindControls(driveRequest, strafeRequest, rotateRequest);
    NamedCommands.registerCommand(Constants.NamedCommands.LIFT_L4_COMMAND_NAME, this.autononomousL4Command());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_ALIGN_COMMAND_NAME, this.autononomousAlignCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SCORE_COMMAND_NAME, this.autonomousScoreCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.WAIT_FOR_INTAKE_COMMAND_NAME, this.autonomousWaitForIntakeCommand());
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
		});
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
			return END_EFFECTOR_SUBSYSTEM.isCoralCentered();
		});
	}

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/state", getState().toString());
  }

  @Override
  public void close() throws Exception {

  }
}
