/* SOMEHOW HELPFUL COMMENTS HERE:
 * Method usages:
 * selectNearestTarget() - Automatically choose closest preset target
 * getDirectionToTarget() - Unit vector direction for simple forward driving
 * getHeadingToTargetDegrees() - Angle robot should rotate toward
 * getTargetPose() - Position we are going toward
 * getCurrentPose() - Pose from Limelight
*/

//package
package frc.robot.subsystems.localization;

//imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;

public class LocalizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight"; // change this to correct name ex change to limelightPH

    // These are the preset field coordinates the robot will try to move to. (Coordinates to move to)

    public static final Pose2d[] TARGETS = new Pose2d[] { //target pose RANDOM AS OF NOW
        new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(90), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(90), Rotation2d.fromDegrees(180))
    };

    // Latest pose estimate from Limelight. Updated every periodic cycle when valid.
    private Pose2d latestPose = new Pose2d();

    // Index into TARGETS array representing which target the robot is currently moving toward.
    private int currentTarget = 0;

    @Override
    public void periodic() {
        // Read the current pose from Limelight (field coordinate system - blue alliance).
        Pose2d p = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);

        // Only update if Limelight returned valid data
        if (p != null && !Double.isNaN(p.getX()) && !Double.isNaN(p.getY())) {
            latestPose = p;
        }
    }

    // Returns the most recent pose estimate received from Limelight.
    public Pose2d getCurrentPose() {
        return latestPose;
    }

    // Manually select which target to drive toward by index number.
    public void setTarget(int i) {
        currentTarget = i;
        if (currentTarget < 0) currentTarget = 0;
        if (currentTarget >= TARGETS.length) currentTarget = TARGETS.length - 1;
    }

    // Returns the current target pose being tracked.
    public Pose2d getTargetPose() {
        return TARGETS[currentTarget];
    }

    // Chooses the closest target to the current robot location.
    // Useful for automatically selecting where to drive next.
    public void selectNearestTarget() {
        int best = 0;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i < TARGETS.length; i++) {
            double d = latestPose.getTranslation().getDistance(TARGETS[i].getTranslation());
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        currentTarget = best;
    }

    // Direction vector pointing from the robot toward the selected target.
    // Always normalized to length 1. Use as drive direction.
    public Translation2d getDirectionToTarget() {
        Pose2d goal = TARGETS[currentTarget];
        Translation2d d = goal.getTranslation().minus(latestPose.getTranslation());
        if (d.getNorm() > 0.001) return d.div(d.getNorm());
        return new Translation2d();
    }

    // Returns the angle in degrees that the robot should rotate to face the target.
    // This is used for simple turning control so robot faces where it is driving.
    public double getHeadingToTargetDegrees() {
        Pose2d goal = TARGETS[currentTarget];
        double dx = goal.getX() - latestPose.getX();
        double dy = goal.getY() - latestPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
