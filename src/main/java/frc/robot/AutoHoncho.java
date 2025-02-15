package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class AutoHoncho {
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

	
}
