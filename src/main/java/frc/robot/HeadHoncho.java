package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class HeadHoncho extends StateMachine implements AutoCloseable {
  public enum State implements SystemState {
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

        if (END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) return INTAKE;

        if (s_regurgitateButton.getAsBoolean() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
          return REGURGITATE;
        }

        if (s_L1Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L1;
        if (s_L2Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L2;
        if (s_L3Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L3;
        if (s_L4Button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) return L4;

        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
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
        if (LIFT_SUBSYSTEM.isAtState(TargetLiftStates.L1) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
        if (s_scoreButton.getAsBoolean()) return SCORE;

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
        if (LIFT_SUBSYSTEM.isAtState(TargetLiftStates.L2) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
        if (s_scoreButton.getAsBoolean()) return SCORE;

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
        if (LIFT_SUBSYSTEM.isAtState(TargetLiftStates.L3) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
        if (s_scoreButton.getAsBoolean()) return SCORE;

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
        if (LIFT_SUBSYSTEM.isAtState(TargetLiftStates.L4) && DRIVE_SUBSYSTEM.isAligned()) return SCORE_L4;
        if (s_scoreButton.getAsBoolean()) return SCORE;

        if (s_L1Button.getAsBoolean()) return L1;
        if (s_L2Button.getAsBoolean()) return L2;
        if (s_L3Button.getAsBoolean()) return L3;
        if (s_L4Button.getAsBoolean()) return L4;

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
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return REST;

        return this;
      }

      @Override
      public void end(boolean interrupted) {
        LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
        END_EFFECTOR_SUBSYSTEM.requestStop();
        DRIVE_SUBSYSTEM.cancelAutoAlign();
      }
    },
    SCORE_L4 {
      @Override
      public void initialize() {
        END_EFFECTOR_SUBSYSTEM.requestScoreReverse();
      }

      @Override
      public SystemState nextState() {
        if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return REST;

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
    BooleanSupplier scoreButton,
    BooleanSupplier cancelButton
  ) {
    s_intakeButton = intakeButton;
    s_regurgitateButton = regurgitateButton;
    s_L1Button = L1Button;
    s_L2Button = L2Button;
    s_L3Button = L3Button;
    s_L4Button = L4Button;

    s_scoreButton = scoreButton;
    s_cancelButton = cancelButton;

    DRIVE_SUBSYSTEM.bindControls(driveRequest, strafeRequest, rotateRequest);
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
