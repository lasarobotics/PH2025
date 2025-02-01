package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class HeadHoncho extends StateMachine implements AutoCloseable{
    public static record Hardware() {}

    public enum State implements SystemState {
        REST {
            @Override
            public void initialize() {
                // reset the whole robot
                END_EFFECTOR_SUBSYSTEM.stop();
                DRIVE_SUBSYSTEM.cancelAutoAlign();
                LIFT_SUBSYSTEM.goToState(STOW);
                INTAKE_SUBSYSTEM.stop();
            }

            @Override
            public SystemState nextState() {
                if (intake_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(STOW)) {
                    return INTAKE;
                }

                if (DRIVE_SUBSYSTEM.nearSource() && END_EFFECTOR_SUBSYSTEM.isEmpty() && LIFT_SUBSYSTEM.isAtState(STOW)) {
                    return AUTO_INTAKE;
                }

                if (regurgitate_button.getAsBoolean() && LIFT_SUBSYSTEM.isAtState(STOW)) {
                    return REGURGITATE;
                }

                if (l1_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.hasCoral()) {
                    return L1;
                }
                if (l2_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.hasCoral()) {
                    return L2;
                }
                if (l3_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.hasCoral()) {
                    return L3;
                }
                if (l4_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.hasCoral()) {
                    return L4;
                }
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                INTAKE_SUBSYSTEM.intake();
                END_EFFECTOR_SUBSYSTEM.intake();
            }

            @Override
            public SystemState nextState() {
                if (END_EFFECTOR_SUBSYSTEM.hasCoral()) return REST;
                if (!intake_button.getAsBoolean() && END_EFFECTOR_SUBSYSTEM.isEmpty()) return REST;
            }

            @Override
            public void end(boolean interrupted) {
                INTAKE_SUBSYSTEM.stop();
                END_EFFECTOR_SUBSYSTEM.stop();
            }
        },
        AUTO_INTAKE {
            @Override
            public void initialize() {
                INTAKE_SUBSYSTEM.intake();
                END_EFFECTOR_SUBSYSTEM.intake();
            }

            @Override
            public SystemState nextState() {
                if (END_EFFECTOR_SUBSYSTEM.hasCoral()) return REST;
            }

            @Override
            public void end(boolean interrupted) {
                INTAKE_SUBSYSTEM.stop();
                END_EFFECTOR_SUBSYSTEM.stop();
            }
        },
        REGURGITATE {
            @Override
            public void initialize() {
                END_EFFECTOR_SUBSYSTEM.regurgitate();
                INTAKE_SUBSYSTEM.regurgitate();
            }

            @Override
            public SystemState nextState() {
                if (!regurgitate_button.getAsBoolean() && (INTAKE_SUBSYSTEM.hasCoral() || INTAKE_SUBSYSTEM.isEmpty())) return REST;
                // TODO algae
            }

            @Override
            public void end(boolean interrupted) {
                INTAKE_SUBSYSTEM.stop();
                END_EFFECTOR_SUBSYSTEM.stop();
            }
        },

        L1 {
            @Override
            public void initialize() {
                LIFT_SUBSYSTEM.goToState(L1);
                DRIVE_SUBSYSTEM.requestAutoAlign();
            }

            @Override
            public SystemState nextState() {
                if (LIFT_SUBSYSTEM.isAtState(L1) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;
            }
        },
        L2 {
            @Override
            public void initialize() {
                LIFT_SUBSYSTEM.goToState(L2);
                DRIVE_SUBSYSTEM.requestAutoAlign();
            }

            @Override
            public SystemState nextState() {
                if (LIFT_SUBSYSTEM.isAtState(L2) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;
            }
        },
        L3 {
            @Override
            public void initialize() {
                LIFT_SUBSYSTEM.goToState(L3);
                DRIVE_SUBSYSTEM.requestAutoAlign();
            }

            @Override
            public SystemState nextState() {
                if (LIFT_SUBSYSTEM.isAtState(L3) && DRIVE_SUBSYSTEM.isAligned()) return SCORE;
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;
            }
        },
        L4 {
            @Override
            public void initialize() {
                LIFT_SUBSYSTEM.goToState(L4);
                DRIVE_SUBSYSTEM.requestAutoAlign();
            }

            @Override
            public SystemState nextState() {
                if (LIFT_SUBSYSTEM.isAtState(L4) && DRIVE_SUBSYSTEM.isAligned()) return SCORE_L4;
                if (score_button.getAsBoolean()) return SCORE;

                if (l1_button.getAsBoolean()) return L1;
                if (l2_button.getAsBoolean()) return L2;
                if (l3_button.getAsBoolean()) return L3;
                if (l4_button.getAsBoolean()) return L4;
            }
        },

        SCORE {
            @Override
            public void initialize() {
                END_EFFECTOR_SUBSYSTEM.outtake();
            }

            @Override
            public SystemState nextState() {
                if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return REST;
            }

            @Override
            public void end(boolean interrupted) {
                LIFT_SUBSYSTEM.goToState(STOW);
                END_EFFECTOR_SUBSYSTEM.stop();
                DRIVE_SUBSYSTEM.cancelAutoAlign();
            }
        },
        SCORE_L4 {
            @Override
            public void initialize() {
                END_EFFECTOR_SUBSYSTEM.outtake_reverse();
            }

            @Override
            public SystemState nextState() {
                if (END_EFFECTOR_SUBSYSTEM.isEmpty()) return REST;
            }

            @Override
            public void end(boolean interrupted) {
                LIFT_SUBSYSTEM.goToState(STOW);
                END_EFFECTOR_SUBSYSTEM.stop();
                DRIVE_SUBSYSTEM.cancelAutoAlign();
            }
        }
    }

    // Subsystems
    private static DriveSubsystem DRIVE_SUBSYSTEM;
    private static IntakeSubsystem INTAKE_SUBSYSTEM;
    private static LiftSubsystem LIFT_SUBSYSTEM;
    private static EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM;
    private static ClimberSubsystem CLIMB_SUBSYSTEM;

    // User input
    private static DoubleSupplier drive_x_request;
    private static DoubleSupplier drive_y_request;
    private static DoubleSupplier drive_rotate_request;

    private static BooleanSupplier intake_button;
    private static BooleanSupplier regurgitate_button;

    private static BooleanSupplier l1_button;
    private static BooleanSupplier l2_button;
    private static BooleanSupplier l3_button;
    private static BooleanSupplier l4_button;

    private static BooleanSupplier score_button; // force robot to score, regardless of alignment

    public HeadHoncho(
        Hardware hardware,
        DriveSubsystem drive_subsystem,
        IntakeSubsystem intake_subsystem,
        LiftSubsystem lift_subsystem,
        EndEffectorSubsystem end_effector_subsystem,
        ClimberSubsystem climb_subsystem
    ) {
        super(State.REST);

        DRIVE_SUBSYSTEM = drive_subsystem;
        INTAKE_SUBSYSTEM = intake_subsystem;
        LIFT_SUBSYSTEM = lift_subsystem;
        END_EFFECTOR_SUBSYSTEM = end_effector_subsystem;
        CLIMB_SUBSYSTEM = climb_subsystem;
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
        BooleanSupplier score_button
    ) {
        HeadHoncho.drive_x_request = drive_x_reqeust;
        HeadHoncho.drive_y_request = drive_y_request;
        HeadHoncho.drive_rotate_request = drive_rotate_request;

        HeadHoncho.intake_button = intake_button;
        HeadHoncho.regurgitate_button = regurgitate_button;
        HeadHoncho.l1_button = l1_button;
        HeadHoncho.l2_button = l2_button;
        HeadHoncho.l3_button = l3_button;
        HeadHoncho.l4_button = l4_button;

        HeadHoncho.score_button = score_button;

        DRIVE_SUBSYSTEM.bindControls(drive_x_reqeust, drive_y_request, drive_rotate_request);
    }

    @Override
    public void close() throws Exception {
        
    }
}
