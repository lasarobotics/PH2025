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
    public static record Hardware() {}

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
                if (intake_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
                    return INTAKE;
                }

                if (END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) return INTAKE;

                if (regurgitate_button.getAsBoolean() && LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW)) {
                    return REGURGITATE;
                }

                if (l1_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) {
                    return L1;
                }
                if (l2_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) {
                    return L2;
                }
                if (l3_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) {
                    return L3;
                }
                if (l4_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isCoralCentered()) {
                    return L4;
                }

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
                if (cancel_button.getAsBoolean()) return REST;

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
                if (cancel_button.getAsBoolean() && (INTAKE_SUBSYSTEM.coralInIntake() || INTAKE_SUBSYSTEM.coralNotInIntake())) return REST;
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
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;

                if (cancel_button.getAsBoolean()) return STOW;

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
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;

                if (cancel_button.getAsBoolean()) return STOW;

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
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;

                if (cancel_button.getAsBoolean()) return STOW;

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
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;

                if (cancel_button.getAsBoolean()) return STOW;

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

    private static BooleanSupplier intake_button;
    private static BooleanSupplier regurgitate_button;

    private static BooleanSupplier l1_button;
    private static BooleanSupplier l2_button;
    private static BooleanSupplier l3_button;
    private static BooleanSupplier l4_button;

    private static BooleanSupplier score_button; // force robot to score, regardless of alignment

    private static BooleanSupplier cancel_button;

    public HeadHoncho(
        Hardware hardware,
        DriveSubsystem drive_subsystem,
        IntakeSubsystem intake_subsystem,
        LiftSubsystem lift_subsystem,
        EndEffectorSubsystem end_effector_subsystem
    ) {
        super(State.REST);

        DRIVE_SUBSYSTEM = drive_subsystem;
        INTAKE_SUBSYSTEM = intake_subsystem;
        LIFT_SUBSYSTEM = lift_subsystem;
        END_EFFECTOR_SUBSYSTEM = end_effector_subsystem;
    }

    public void bindControls(
        DoubleSupplier drive_x_reqeust,
        DoubleSupplier drive_y_request,
        DoubleSupplier drive_rotate_request,
        BooleanSupplier intake_button,
        BooleanSupplier regurgitate_button,
        BooleanSupplier l1_button,
        BooleanSupplier l2_button,
        BooleanSupplier l3_button,
        BooleanSupplier l4_button,
        BooleanSupplier score_button,
        BooleanSupplier cancel_button
    ) {
        HeadHoncho.intake_button = intake_button;
        HeadHoncho.regurgitate_button = regurgitate_button;
        HeadHoncho.l1_button = l1_button;
        HeadHoncho.l2_button = l2_button;
        HeadHoncho.l3_button = l3_button;
        HeadHoncho.l4_button = l4_button;

        HeadHoncho.score_button = score_button;

        HeadHoncho.cancel_button = cancel_button;

        DRIVE_SUBSYSTEM.bindControls(drive_x_reqeust, drive_y_request, drive_rotate_request);
    }

    public static Hardware initializeHardware() {
        return new Hardware();
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
