package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class LiftInstructions {
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
    static final Distance STOW_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(0.05));

    static final Distance L1_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(1.167969)).minus(Inches.of(1.375));
    static final Distance L2_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(2.55246)).plus(Inches.of(1)).minus(Inches.of(1.375));
    static final Distance CLEAR_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(3.824)).minus(Inches.of(1.375));
    static final Distance L3_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(0));
    static final Distance L4_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(4.49)).minus(Inches.of(1.625));
    static final Distance PANIC_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(4.5)).minus(Inches.of(1.125));
    static final Distance TURBO_HEIGHT = L4_HEIGHT;
    static final Distance A1_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(0.45)).minus(Inches.of(0.375));
    static final Distance A2_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(2.65678)).minus(Inches.of(0.375));
    
    interface ArmComparison {
        boolean compare(Angle target);
    }

    interface ElevatorComparison {
        boolean compare(Distance target);
    }

    public record IDF (
        Angle wantedArmAngle,
        ArmComparison armComparison,
        Distance wantedElevatorHeight,
        ElevatorComparison elevatorComparison
    ) {}
    
    public static IDF[] STOW_TURBO_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            TURBO_HEIGHT,
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            TURBO_ANGLE,
            (s) -> s.isNear(TURBO_ANGLE, ARM_TOLERANCE),
            TURBO_HEIGHT,
            (s) -> s.isNear(TURBO_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_L1_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L1_ANGLE,
            (s) -> s.isNear(SCORING_L1_ANGLE, ARM_TOLERANCE),
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_L2_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L2_ANGLE,
            (s) -> s.isNear(SCORING_L2_ANGLE, ARM_TOLERANCE),
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_L3_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT,
            (s) -> s.isNear(CLEAR_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SAFE_REEF_ANGLE_TOP,
            (s) -> s.gte(SAFE_INTAKE_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L3_ANGLE,
            (s) -> s.isNear(SCORING_L3_ANGLE, ARM_TOLERANCE),
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_L4_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L4_HEIGHT,
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> s.isNear(SCORING_L4_ANGLE, ARM_TOLERANCE),
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_A1_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            A1_HEIGHT,
            (s) -> s.lte(A1_HEIGHT)
        ),
        new IDF(
            SCORING_A1_ANGLE,
            (s) -> s.isNear(SCORING_A1_ANGLE, ARM_TOLERANCE),
            A1_HEIGHT,
            (s) -> s.isNear(A1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] STOW_A2_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            A2_HEIGHT,
            (s) -> s.lte(A2_HEIGHT)
        ),
        new IDF(
            SCORING_A2_ANGLE,
            (s) -> s.isNear(SCORING_A2_ANGLE, ARM_TOLERANCE),
            A2_HEIGHT,
            (s) -> s.isNear(A2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L1_STOW_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            STOW_ANGLE,
            (s) -> s.isNear(STOW_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L1_L2_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L2_ANGLE,
            (s) -> s.isNear(SCORING_L2_ANGLE, ARM_TOLERANCE),
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L1_L3_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM,
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT,
            (s) -> s.isNear(CLEAR_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SAFE_REEF_ANGLE_TOP,
            (s) -> s.gte(SAFE_INTAKE_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L3_ANGLE,
            (s) -> s.isNear(SCORING_L3_ANGLE, ARM_TOLERANCE),
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L1_L4_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM,
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L4_HEIGHT,
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> s.isNear(SCORING_L4_ANGLE, ARM_TOLERANCE),
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L1_TURBO_INSTRUCTIONS = new IDF[]{
        new IDF(
            TURBO_ANGLE.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            TURBO_ANGLE,
            (s) -> s.isNear(TURBO_ANGLE, ARM_TOLERANCE),
            TURBO_HEIGHT,
            (s) -> s.isNear(TURBO_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    // they are the same thing
    public static IDF[] L2_STOW_INSTRUCTIONS = L1_STOW_INSTRUCTIONS;

    public static IDF[] L2_L1_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L1_ANGLE,
            (s) -> s.isNear(SCORING_L1_ANGLE, ARM_TOLERANCE),
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L2_L3_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT,
            (s) -> s.isNear(CLEAR_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SAFE_REEF_ANGLE_TOP,
            (s) -> s.gte(SAFE_INTAKE_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L3_ANGLE,
            (s) -> s.isNear(SCORING_L3_ANGLE, ARM_TOLERANCE),
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L2_L4_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L4_HEIGHT,
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> s.isNear(SCORING_L4_ANGLE, ARM_TOLERANCE),
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L2_TURBO_INSTRUCTIONS = new IDF[]{
        new IDF(
            TURBO_ANGLE.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            TURBO_ANGLE,
            (s) -> s.isNear(TURBO_ANGLE, ARM_TOLERANCE),
            TURBO_HEIGHT,
            (s) -> s.isNear(TURBO_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    // Levi was here
    
    public static IDF[] L3_STOW_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_TOP.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_REEF_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE),
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            STOW_ANGLE,
            (s) -> s.isNear(STOW_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L3_L1_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_TOP,
            (s) -> s.lte(SAFE_REEF_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE),
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM,
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L1_ANGLE,
            (s) -> s.isNear(SCORING_L1_ANGLE, ARM_TOLERANCE),
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L3_L2_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_TOP,
            (s) -> s.lte(SAFE_REEF_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE),
            (s) -> s.gte(CLEAR_HEIGHT)
        ),
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L2_ANGLE,
            (s) -> s.isNear(SCORING_L2_ANGLE, ARM_TOLERANCE),
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L3_L4_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_REEF_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> s.isNear(SCORING_L4_ANGLE, ARM_TOLERANCE),
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L3_TURBO_INSTRUCTIONS = new IDF[]{
        new IDF(
            null,
            null,
            TURBO_HEIGHT,
            (s) -> s.gte(TURBO_HEIGHT)
        ),
        new IDF(
            TURBO_ANGLE.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            TURBO_ANGLE,
            (s) -> s.isNear(TURBO_ANGLE, ARM_TOLERANCE),
            TURBO_HEIGHT,
            (s) -> s.isNear(TURBO_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_STOW_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            STOW_ANGLE,
            (s) -> s.isNear(STOW_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_L1_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM,
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L1_ANGLE,
            (s) -> s.isNear(SCORING_L1_ANGLE, ARM_TOLERANCE),
            L1_HEIGHT,
            (s) -> s.isNear(L1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_L2_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_BOTTOM,
            (s) -> s.lte(SAFE_INTAKE_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L2_ANGLE,
            (s) -> s.isNear(SCORING_L2_ANGLE, ARM_TOLERANCE),
            L2_HEIGHT,
            (s) -> s.isNear(L2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_L3_INSTRUCTIONS = new IDF[]{
        new IDF(
            SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE),
            (s) -> s.lte(SAFE_REEF_ANGLE_TOP),
            null,
            null
        ),
        new IDF(
            null,
            null,
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        ),
        new IDF(
            SCORING_L3_ANGLE,
            (s) -> s.isNear(SCORING_L3_ANGLE, ARM_TOLERANCE),
            L3_HEIGHT,
            (s) -> s.isNear(L3_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_TURBO_INSTRUCTIONS = new IDF[]{
        new IDF(
            TURBO_ANGLE.plus(ARM_TOLERANCE),
            (s) -> s.gte(SAFE_REEF_ANGLE_BOTTOM),
            null,
            null
        ),
        new IDF(
            TURBO_ANGLE,
            (s) -> s.isNear(TURBO_ANGLE, ARM_TOLERANCE),
            TURBO_HEIGHT,
            (s) -> s.isNear(TURBO_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] L4_PANIC_INSTRUCTIONS = new IDF[]{
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> { return true; },
            PANIC_HEIGHT,
            (s) -> { return true; }
        )
    };

    // these are all the same
    public static IDF[] TURBO_STOW_INSTRUCTIONS =  L4_STOW_INSTRUCTIONS;
    public static IDF[] TURBO_L1_INSTRUCTIONS = L4_L1_INSTRUCTIONS;
    public static IDF[] TURBO_L2_INSTRUCTIONS = L4_L2_INSTRUCTIONS;
    public static IDF[] TURBO_L3_INSTRUCTIONS = STOW_L3_INSTRUCTIONS;

    public static IDF[] TURBO_L4_INSTRUCTIONS = new IDF[]{
        new IDF(
            SCORING_L4_ANGLE,
            (s) -> s.isNear(SCORING_L4_ANGLE, ARM_TOLERANCE),
            L4_HEIGHT,
            (s) -> s.isNear(L4_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    // they are the same
    public static IDF[] A1_STOW_INSTRUCTIONS = L1_STOW_INSTRUCTIONS;

    public static IDF[] A1_A2_INSTRUCTIONS = new IDF[]{
        new IDF(
            null,
            null,
            A2_HEIGHT,
            (s) -> s.lte(A2_HEIGHT)
        ),
        new IDF(
            SCORING_A2_ANGLE,
            (s) -> s.isNear(SCORING_A2_ANGLE, ARM_TOLERANCE),
            A2_HEIGHT,
            (s) -> s.isNear(A2_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] A1_A_SCORE_INSTRUCTIONS = new IDF[]{
        new IDF(
            SCORING_A_ANGLE,
            (s) -> s.isNear(SCORING_A_ANGLE, ARM_TOLERANCE),
            null,
            null
        ),
        new IDF(
            null,
            null,
            STOW_HEIGHT,
            (s) -> s.lte(STOW_HEIGHT)
        ),
        new IDF(
            SCORING_A_ANGLE,
            (s) -> s.isNear(SCORING_A_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    // they are the same
    public static IDF[] A2_STOW_INSTRUCTIONS = L1_STOW_INSTRUCTIONS;

    public static IDF[] A2_A1_INSTRUCTIONS = new IDF[]{
        new IDF(
            null,
            null,
            A1_HEIGHT,
            (s) -> s.lte(A1_HEIGHT)
        ),
        new IDF(
            SCORING_A1_ANGLE,
            (s) -> s.isNear(SCORING_A1_ANGLE, ARM_TOLERANCE),
            A1_HEIGHT,
            (s) -> s.isNear(A1_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] A2_A_SCORE_INSTRUCTIONS = new IDF[]{
        new IDF(
            SCORING_A_ANGLE,
            (s) -> s.isNear(SCORING_A_ANGLE, ARM_TOLERANCE),
            null,
            null
        ),
        new IDF(
            null,
            null,
            STOW_HEIGHT,
            (s) -> s.lte(STOW_HEIGHT)
        ),
        new IDF(
            SCORING_A_ANGLE,
            (s) -> s.isNear(SCORING_A_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };

    public static IDF[] A_SCORE_STOW_INSTRUCTIONS = new IDF[]{
        new IDF(
            STOW_ANGLE,
            (s) -> s.isNear(STOW_ANGLE, ARM_TOLERANCE),
            STOW_HEIGHT,
            (s) -> s.isNear(STOW_HEIGHT, ELEVATOR_TOLERANCE)
        )
    };


  /**
   * Maps a starting lift state and an ending lift state to an instruction set.
   * 
   * @param startingState The starting state
   * @param endingState The ending state
   * @return The instruction set representing the transition from startingState to endingState. If there is no valid transition, return null
   */
  public static IDF[] mapStatesToTransition(TargetLiftStates startingState, TargetLiftStates endingState) {
    // shortcut identical case
    if (startingState == endingState) return null;
    switch (startingState) {
        case STOW:
            switch (endingState) {
                case TURBO:
                    return STOW_TURBO_INSTRUCTIONS;
                case L1:
                    return STOW_L1_INSTRUCTIONS;
                case L2:
                    return STOW_L2_INSTRUCTIONS;
                case L3:
                    return STOW_L3_INSTRUCTIONS;
                case L4:
                    return STOW_L4_INSTRUCTIONS;
                case A1:
                    return STOW_A1_INSTRUCTIONS;
                case A2:
                    return STOW_A2_INSTRUCTIONS;
                default:
                    return null;
            }
        case L1:
            switch (endingState) {
                case STOW:
                    return L1_STOW_INSTRUCTIONS;
                case L2:
                    return L1_L2_INSTRUCTIONS;
                case L3:
                    return L1_L3_INSTRUCTIONS;
                case L4:
                    return L1_L4_INSTRUCTIONS;
                case TURBO:
                    return L1_TURBO_INSTRUCTIONS;
                default:
                    return null;
            }
        case L2:
            switch (endingState) {
                case STOW:
                    return L2_STOW_INSTRUCTIONS;
                case L1:
                    return L2_L1_INSTRUCTIONS;
                case L3:
                    return L2_L3_INSTRUCTIONS;
                case L4:
                    return L2_L4_INSTRUCTIONS;
                case TURBO:
                    return L2_TURBO_INSTRUCTIONS;
                default:
                    return null;
            }
        case L3:
            switch (endingState) {
                case STOW:
                    return L3_STOW_INSTRUCTIONS;
                case L1:
                    return L3_L1_INSTRUCTIONS;
                case L2:
                    return L3_L2_INSTRUCTIONS;
                case L4:
                    return L3_L4_INSTRUCTIONS;
                case TURBO:
                    return L3_TURBO_INSTRUCTIONS;
                default:
                    return null;
            }
        case L4:
            switch (endingState) {
                case STOW:
                    return L4_STOW_INSTRUCTIONS;
                case L1:
                    return L4_L1_INSTRUCTIONS;
                case L2:
                    return L4_L2_INSTRUCTIONS;
                case L3:
                    return L4_L3_INSTRUCTIONS;
                case TURBO:
                    return L4_TURBO_INSTRUCTIONS;
                case PANIC:
                    return L4_PANIC_INSTRUCTIONS;
                default:
                    return null;
            }
        case TURBO:
            switch (endingState) {
                case STOW:
                    return TURBO_STOW_INSTRUCTIONS;
                case L1:
                    return TURBO_L1_INSTRUCTIONS;
                case L2:
                    return TURBO_L2_INSTRUCTIONS;
                case L3:
                    return TURBO_L3_INSTRUCTIONS;
                case L4:
                    return TURBO_L4_INSTRUCTIONS;
                default:
                    return null;
            }
        case A1:
            switch (endingState) {
                case STOW:
                    return A1_STOW_INSTRUCTIONS;
                case A2:
                    return A1_A2_INSTRUCTIONS;
                case A_SCORE:
                    return A1_A_SCORE_INSTRUCTIONS;
                default:
                    return null;
            }
        case A2:
            switch (endingState) {
                case STOW:
                    return A2_STOW_INSTRUCTIONS;
                case A1:
                    return A2_A1_INSTRUCTIONS;
                case A_SCORE:
                    return A2_A_SCORE_INSTRUCTIONS;
                default:
                    return null;
            }
        case A_SCORE:
            switch (endingState) {
                case STOW:
                    return A_SCORE_STOW_INSTRUCTIONS;
                default:
                    return null;
            }
        case PANIC:
            switch (endingState) {
                case STOW:
                    return L4_STOW_INSTRUCTIONS;
                case L1:
                    return L4_L1_INSTRUCTIONS;
                case L2:
                    return L4_L2_INSTRUCTIONS;
                case L3:
                    return L4_L3_INSTRUCTIONS;
                case TURBO:
                    return L4_TURBO_INSTRUCTIONS;
                default:
                    return null;
            }
        default:
            return null;
    }
  }
}
