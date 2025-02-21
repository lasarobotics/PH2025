package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.EndEffectorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem.TargetLiftStates;

public class AutoHoncho implements Subsystem {
	// Subsystems
  private static DriveSubsystem DRIVE_SUBSYSTEM;
  private static IntakeSubsystem INTAKE_SUBSYSTEM;
  private static LiftSubsystem LIFT_SUBSYSTEM;
  private static EndEffectorSubsystem END_EFFECTOR_SUBSYSTEM;

	public AutoHoncho(
		DriveSubsystem driveSubsystem,
    IntakeSubsystem intakeSubsystem,
    LiftSubsystem liftSubsystem,
    EndEffectorSubsystem endEffectorSubsystem
	) {
		DRIVE_SUBSYSTEM = driveSubsystem;
		INTAKE_SUBSYSTEM = intakeSubsystem;
		LIFT_SUBSYSTEM = liftSubsystem;
		END_EFFECTOR_SUBSYSTEM = endEffectorSubsystem;
	}

	/**
	 * Tells the robot to move the lift to the intake state during autonomous
	 * @return Command which tells the robot to move the lift to the intake state during autonomous
	 */
	private Command autononomousMoveLiftToStowCommand() {
		return Commands.startEnd(() -> 
		{
			LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
		}, () -> {},
		LIFT_SUBSYSTEM
		)
		.until(() -> {
			return LIFT_SUBSYSTEM.isAtState(TargetLiftStates.STOW);
		});
	}

	/**
	 * Sets the intake and end effector subsystems to intake state in autonomous
	 * @return Command which sets the intake and end-effector to the intake state in autonomous
	 */
	private Command autonomousWaitForIntakeCommand() {
		return Commands.startEnd(() -> 
		{}, () -> {
			LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
		},
		INTAKE_SUBSYSTEM, END_EFFECTOR_SUBSYSTEM
		)
		.until(() -> {
			return END_EFFECTOR_SUBSYSTEM.isCoralCentered();
		});
	}


	/**
	 * Tells the robot to move the lift to the L4 state during autonomous
	 * @return Command which tells the robot to move the lift to the L4 state during autonomous
	 */
	private Command autononomousL4Command() {
		return Commands.startEnd(() -> 
		{
			LIFT_SUBSYSTEM.setState(TargetLiftStates.L4);
		}, () -> {},
		LIFT_SUBSYSTEM
		)
		.until(() -> {
			return LIFT_SUBSYSTEM.isAtState(TargetLiftStates.L4);
		});
	}

	/**
	 * Tells the robot to score the coral during autonomous
	 * @return Command that tells the robot to score coral during autononomous
	 */
	private Command autonomousScoreCommand() { {
		return 
		Commands.sequence(
			autononomousL4Command(),
		Commands.startEnd(() ->
		{
			END_EFFECTOR_SUBSYSTEM.setState(EndEffectorStates.SCORE_L4);
		}, () -> {
			LIFT_SUBSYSTEM.setState(TargetLiftStates.STOW);
			INTAKE_SUBSYSTEM.startIntake();
			END_EFFECTOR_SUBSYSTEM.requestIntake();
		},
		END_EFFECTOR_SUBSYSTEM)
		.until(() -> {
			return END_EFFECTOR_SUBSYSTEM.isEmpty();
		}));
	}
 }
}
