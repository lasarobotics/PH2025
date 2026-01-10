package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.State;
import frc.robot.lib.StateMachine;

public final class LiftSubsystem extends StateMachine {

  public static enum TargetLiftStates {
    NOTHING,
    TURBO,
    STOW,
    L1,
    L2,
    L3,
    L4,
    PANIC,
    A1,
    A2,
    A_SCORE,
    A_KICK
  }

  private static TargetLiftStates nextState;
  private static TargetLiftStates curState;
  private static boolean isLiftReady;
  private static boolean isDisabled;
  static final Distance HOMING_EPSILON = Millimeters.of(5);

  // Tolerance in cm of top and bottom minimum clearance
  static final Distance ELEVATOR_TOLERANCE = Inches.of(2.0);

  // Tolerance in degrees of arm
  static final Angle ARM_TOLERANCE = Degrees.of(2.0);

  static final Angle SAFE_REEF_ANGLE_BOTTOM = Rotations.of(-0.256836);
  static final Angle SAFE_REEF_ANGLE_TOP = Rotations.of(0.287598-0.0278);
  static final Angle SAFE_INTAKE_ANGLE_BOTTOM =  Rotations.of(-0.24469);
  static final Angle SAFE_INTAKE_ANGLE_TOP = Rotations.of(0.109375);
  static final Angle SAFE_NEW_ANGLE_INTAKE = Rotations.of(0.104004);

  static final Angle SCORING_L1_ANGLE = Rotations.of(-0.263184);
  static final Angle SCORING_L2_ANGLE = Rotations.of(-0.27002);
  static final Angle SCORING_L3_ANGLE = Rotations.of(0.34375);
  static final Angle SCORING_L4_ANGLE = Rotations.of(0.326172-0.027777777777).plus(Degrees.of(7));
  static final Angle SCORING_A1_ANGLE = Rotations.of(-0.375);
  static final Angle SCORING_A2_ANGLE = Rotations.of(-0.375);
  static final Angle SCORING_A_ANGLE = Rotations.of(-0.256836);
  static final Angle TURBO_ANGLE = SAFE_INTAKE_ANGLE_BOTTOM;

  static final Angle STOW_ANGLE = Rotations.of(-0.215333);
  static final Distance STOW_HEIGHT = LiftHardware.convertToDistance(Rotations.of(0.05));

  static final Distance L1_HEIGHT = LiftHardware.convertToDistance(Rotations.of(1.167969)).minus(Inches.of(1.375));
  static final Distance L2_HEIGHT = LiftHardware.convertToDistance(Rotations.of(2.55246)).plus(Inches.of(1)).minus(Inches.of(1.375));
  static final Distance CLEAR_HEIGHT = LiftHardware.convertToDistance(Rotations.of(3.824)).minus(Inches.of(1.375));
  static final Distance L3_HEIGHT = LiftHardware.convertToDistance(Rotations.of(0));
  static final Distance L4_HEIGHT = LiftHardware.convertToDistance(Rotations.of(4.49)).minus(Inches.of(1.625));
  static final Distance PANIC_HEIGHT = LiftHardware.convertToDistance(Rotations.of(4.5)).minus(Inches.of(1.125));
  static final Distance TURBO_HEIGHT = L4_HEIGHT;
  static final Distance A1_HEIGHT = LiftHardware.convertToDistance(Rotations.of(0.45)).minus(Inches.of(0.375));
  static final Distance A2_HEIGHT = LiftHardware.convertToDistance(Rotations.of(2.65678)).minus(Inches.of(0.375));

  static final Distance BEAM_BREAK_HEIGHT = LiftHardware.convertToDistance(Rotations.of(0));
  
  private static LiftSubsystem s_instance;

  private LiftSubsystem() {
    super(LiftStates.STOW);
  }

  public static LiftSubsystem getInstance() {
    if (LiftSubsystem.s_instance == null) {
      LiftSubsystem.s_instance = new LiftSubsystem();
    }
    return LiftSubsystem.s_instance;
  }

  /**
   * Return whether the lift is ready to move or not
   * @return Boolean of if lift is at one of the 5 stages
   */
  public boolean isLiftReady() {
    return isLiftReady;
  }

  /**
   * Set state of lift state machine for API purposes
   * @param state The target TargetLiftStates state to go to
   */
  public void setState(TargetLiftStates state) {
    nextState = state;
  }

  /**
   * See if the current nextState is at a given state
   *
   * @param state The LiftStates state to check against
   */
  public boolean isAtState(TargetLiftStates state) {
    return curState == state;
  }

  public enum LiftStates implements State {
    DISABLED {
      @Override
      public void initialize() {
        LiftHardware.stopElevator();
        LiftHardware.stopArm();
        isLiftReady = true; // Done so that the entire robot can keep working, even though the lift is disabled
      }
    },
    IDLE {
      @Override
      public LiftStates nextState() {
        if (nextState == TargetLiftStates.STOW) {
          return STOW;
        }
        return this;
      }
    },
    HOME {

      private boolean isDoneHoming = false;

      @Override
      public void initialize() {
        if (LiftHardware.getArmAngle().lte(STOW_ANGLE.plus(ARM_TOLERANCE)) && LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE)) && LiftHardware.elevatorAtHome()) {
          LiftHardware.startHomingElevator();
        } else {
          isDisabled = true;
        }
      }

      @Override
      public void execute() {
        if (!LiftHardware.elevatorAtHome()) { // There is a ! on this line.
          LiftHardware.setElevatorEncoder(BEAM_BREAK_HEIGHT);
          isDoneHoming = true;
        }
      }

      @Override
      public void end(State nextState) {
        LiftHardware.stopElevator();
      }

      @Override
      public LiftStates nextState() {
        if (isDisabled) {
          return DISABLED;
        }
        if (isDoneHoming) {
          return IDLE;
        }
        return this;
      }
    },
    STOW {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(STOW_HEIGHT);
        LiftHardware.setArmAngle(STOW_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(STOW_ANGLE) && LiftHardware.elevatorAt(STOW_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.STOW;
        if(!isLiftReady) {
          return this;
        }
        if (nextState == TargetLiftStates.L1) {
          return STOW_L1_S0;
        }
        if (nextState == TargetLiftStates.L2) {
          return STOW_L2_S0;
        }
        if (nextState == TargetLiftStates.L3) {
          return STOW_L3_S0;
        }
        if (nextState == TargetLiftStates.L4) {
          return STOW_L4_S0;
        }
        if (nextState == TargetLiftStates.A1) {
          return STOW_A1_S1;
        }
        if (nextState == TargetLiftStates.A2) {
          return STOW_A2_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return STOW_TURBO_S1;
        }
        if (nextState == TargetLiftStates.A_KICK) {
          return A_KICK;
        }
        return this;
      }
    },
    STOW_TURBO_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_TURBO_S2;
        }
        return this;
      }
    },
    STOW_TURBO_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(TURBO_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return TURBO;
        }
        return this;
      }
    },
    TURBO {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(TURBO_ANGLE);
        LiftHardware.setElevatorHeight(TURBO_HEIGHT);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(TURBO_ANGLE) && LiftHardware.elevatorAt(TURBO_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.TURBO;
        if (nextState == TargetLiftStates.L1) {
          return L4_L1_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L4_L2_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return STOW_L3_S2;
        }
        if (nextState == TargetLiftStates.L4) {
          return L4;
        }
        if (nextState == TargetLiftStates.STOW) {
          return L4_STOW_S1;
        }
        return this;
      }
    },
    L1_TURBO_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(TURBO_ANGLE.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return TURBO;
        }
        return this;
      }
    },
    L2_TURBO_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(TURBO_ANGLE.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return TURBO;
        }
        return this;
      }
    },
    L3_TURBO_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setElevatorHeight(TURBO_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(TURBO_HEIGHT)) {
          return L3_TURBO_S2;
        }
        return this;
      }
    },
    L3_TURBO_S2 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(TURBO_ANGLE.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return TURBO;
        }
        return this;
      }
    },
    L4_TURBO_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(TURBO_ANGLE.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return TURBO;
        }
        return this;
      }
    },
    STOW_A1_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_A1_S2;
        }
        return this;
      }
    },
    STOW_A1_S2 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setElevatorHeight(A1_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(A1_HEIGHT)) {
          return A1;
        }
        return this;
      }
    },
    A1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(A1_HEIGHT);
        LiftHardware.setArmAngle(SCORING_A1_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_A1_ANGLE) && LiftHardware.elevatorAt(A1_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.A1;
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.A2) {
          return A2;
        }
        if (nextState == TargetLiftStates.A_SCORE) {
          return A_SCORE;
        }
        return this;
      }
    },
    A_SCORE {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(STOW_HEIGHT);
        LiftHardware.setArmAngle(SCORING_A_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_A_ANGLE) && LiftHardware.elevatorAt(STOW_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.A_SCORE;
        if (nextState == TargetLiftStates.STOW) {
          return STOW;
        }
        if (nextState == TargetLiftStates.A_KICK) {
          return A_KICK;
        }
        return this;
      }
    },
    A_KICK {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(STOW_HEIGHT);
        LiftHardware.setArmAngle(SCORING_A1_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_A1_ANGLE) && LiftHardware.elevatorAt(STOW_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.A_KICK;
        if (nextState == TargetLiftStates.STOW) {
          return STOW;
        }
        return this;
      }
    },
    STOW_A2_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_A2_S2;
        }
        return this;
      }
    },
    STOW_A2_S2 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setElevatorHeight(A2_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(A2_HEIGHT)) {
          return A2;
        }
        return this;
      }
    },
    A2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(A2_HEIGHT);
        LiftHardware.setArmAngle(SCORING_A2_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_A2_ANGLE) && LiftHardware.elevatorAt(A2_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.A2;
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.A1) {
          return A1;
        }
        if (nextState == TargetLiftStates.A_SCORE) {
          return A_SCORE;
        }
        return this;
      }
    },
    STOW_L1_S0 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    STOW_L1_S1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L1_HEIGHT);
        LiftHardware.setArmAngle(SCORING_L1_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_L1_ANGLE) && LiftHardware.elevatorAt(L1_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.L1;
        if(!isLiftReady) {
          return this;
        }
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L1_L2_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L1_L3_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L1_L4_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return L1_TURBO_S1;
        }
        return this;
      }
    },
    L1_STOW_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return L1_STOW_S2;
        }
        return this;
      }
    },
    L1_STOW_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(STOW_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(STOW_HEIGHT)) {
          return STOW;
        }
        return this;
      }
    },
    L1_L2_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return L1_L2_S2;
        }
        return this;
      }
    },
    L1_L2_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L1_L3_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE))) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L1_L4_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE))) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L2_S0 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L2_S1;
        }
        return this;
      }
    },
    STOW_L2_S1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L2_HEIGHT);
        LiftHardware.setArmAngle(SCORING_L2_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_L2_ANGLE) && LiftHardware.elevatorAt(L2_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.L2;
        if(!isLiftReady) {
          return this;
        }
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L2_L1_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L2_L3_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L2_L4_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return L2_TURBO_S1;
        }
        return this;
      }
    },
    L2_L1_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    L2_L3_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L2_L4_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L3_S0 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    STOW_L3_S1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(CLEAR_HEIGHT)) {
          return STOW_L3_S2;
        }
        return this;
      }
    },
    STOW_L3_S2 {
      @Override
      public void initialize() {
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_TOP);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().gte(SAFE_INTAKE_ANGLE_TOP.minus(ARM_TOLERANCE))) {
          return STOW_L3_S3;
        }
        return this;
      }
    },
    STOW_L3_S3 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L3_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L3_HEIGHT)) {
          return L3;
        }
        return this;
      }
    },
    L3 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L3_HEIGHT);
        LiftHardware.setArmAngle(SCORING_L3_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_L3_ANGLE) && LiftHardware.elevatorAt(L3_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.L3;
        if(!isLiftReady) {
          return this;
        }
        if (nextState == TargetLiftStates.STOW) {
          return L3_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L3_L1_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L3_L2_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L3_L4_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return L3_TURBO_S1;
        }
        return this;
      }
    },
    L3_STOW_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_STOW_S2;
        }
        return this;
      }
    },
    L3_STOW_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_STOW_S3;
        }
        return this;
      }
    },
    L3_STOW_S3 {
      @Override
      public void initialize() {
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L3_STOW_S4;
        }
        return this;
      }
    },
    L3_STOW_S4 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(STOW_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(STOW_HEIGHT)) {
          return STOW;
        }
        return this;
      }
    },
    L3_L1_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_TOP);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L1_S2;
        }
        return this;
      }
    },
    L3_L1_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_L1_S3;
        }
        return this;
      }
    },
    L3_L1_S3 {
      @Override
      public void initialize() {
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L3_L1_S4;
        }
        return this;
      }
    },
    L3_L1_S4 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L3_L2_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_TOP);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L2_S2;
        }
        return this;
      }
    },
    L3_L2_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_L2_S3;
        }
        return this;
      }
    },
    L3_L2_S3 {
      @Override
      public void initialize() {
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_BOTTOM)) {
          return L3_L2_S4;
        }
        return this;
      }
    },
    L3_L2_S4 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L3_L4_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L4_S2;
        }
        return this;
      }
    },
    L3_L4_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L4_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L4_HEIGHT)) {
          return L4;
        }
        return this;
      }
    },
    STOW_L4_S0 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L4_S1 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L4_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L4;
        }
        return this;
      }
    },
    L4 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L4_HEIGHT);
        LiftHardware.setArmAngle(SCORING_L4_ANGLE);
      }

      @Override
      public void execute() {
        if (LiftHardware.armAt(SCORING_L4_ANGLE) && LiftHardware.elevatorAt(L4_HEIGHT)) {
          isLiftReady = true;
        } else {
          isLiftReady = false;
        }
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.L4;
        if(!isLiftReady) {
          return this;
        }
        if (nextState == TargetLiftStates.STOW) {
          return L4_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L4_L1_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L4_L2_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L4_L3_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return L4_TURBO_S1;
        }
        if (nextState == TargetLiftStates.PANIC) {
          return PANIC;
        }
        return this;
      }
    },
    PANIC {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(PANIC_HEIGHT);
        LiftHardware.setArmAngle(SCORING_L4_ANGLE);
      }

      @Override
      public void execute() {
        isLiftReady = true;
      }

      @Override
      public State nextState() {
        curState = TargetLiftStates.PANIC;
        if (nextState == TargetLiftStates.STOW) {
          return L4_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L4_L1_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L4_L2_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L4_L3_S1;
        }
        if (nextState == TargetLiftStates.TURBO) {
          return L4_TURBO_S1;
        }
        return this;
      }
    },
    L4_STOW_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_STOW_S2;
        }
        return this;
      }
    },
    L4_STOW_S2 {
      @Override
      public void initialize() {
        LiftHardware.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L1_STOW_S2;
        }
        return this;
      }
    },
    L4_L1_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_L1_S2;
        }
        return this;
      }
    },
    L4_L1_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L4_L2_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_L2_S2;
        }
        return this;
      }
    },
    L4_L2_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L4_L3_S1 {
      @Override
      public void initialize() {
        isLiftReady = false;
        LiftHardware.setArmAngle(SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public State nextState() {
        if (LiftHardware.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L4_L3_S2;
        }
        return this;
      }
    },
    L4_L3_S2 {
      @Override
      public void initialize() {
        LiftHardware.setElevatorHeight(L3_HEIGHT);
      }

      @Override
      public State nextState() {
        if (LiftHardware.elevatorAt(L3_HEIGHT)) {
          return L3;
        }
        return this;
      }
    };
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/isLiftReady", this.isLiftReady());
  }
}
