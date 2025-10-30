package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class GoToPoseHolonomic extends Command {
    private final DriveSubsystem drive;
    private final Pose2d targetPose;

    private final PIDController xController = new PIDController(1.5, 0, 0);
    private final PIDController yController = new PIDController(1.5, 0, 0);
    private final PIDController thetaController = new PIDController(2.0, 0, 0);

    private static final double POSITION_TOLERANCE = 0.05; // 5 cm
    private static final double ANGLE_TOLERANCE = Math.toRadians(3); // 3 degrees

    public GoToPoseHolonomic(DriveSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        System.out.println("[GoToPoseHolonomic] Moving toward " + targetPose);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();

        // Compute translational errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();

        // Compute rotation error
        double headingError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

        // PID corrections
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double omega = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Clamp speeds to sane limits (m/s, rad/s)
        xSpeed = Math.max(Math.min(xSpeed, 2.0), -2.0);
        ySpeed = Math.max(Math.min(ySpeed, 2.0), -2.0);
        omega = Math.max(Math.min(omega, 3.0), -3.0);

        // Drive using the new DriveSubsystem method
        drive.driveFieldRelative(xSpeed, ySpeed, omega);

        // Debug output
        System.out.printf("[GoToPoseHolonomic] Xerr: %.2f  Yerr: %.2f  HeadingErr: %.2f%n",
                xError, yError, Math.toDegrees(headingError));
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = drive.getPose();
        Translation2d targetTranslation = targetPose.getTranslation();
        double distError = pose.getTranslation().getDistance(targetTranslation);
        double angleError = Math.abs(targetPose.getRotation().getRadians() - pose.getRotation().getRadians());
        return distError < POSITION_TOLERANCE && angleError < ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        System.out.println("[GoToPoseHolonomic] " + (interrupted ? "Interrupted" : "Completed"));
    }
}
