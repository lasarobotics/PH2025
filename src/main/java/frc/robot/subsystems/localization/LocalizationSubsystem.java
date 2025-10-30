package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;

public class LocalizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight";

    // Replace with your real targets (meters and final heading)
    public static final Pose2d[] TARGETS = new Pose2d[] {
        new Pose2d(Units.inchesToMeters(40),  Units.inchesToMeters(30),  Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(80),  Units.inchesToMeters(30),  Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30),  Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(30),  Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(40),  Units.inchesToMeters(90),  Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(80),  Units.inchesToMeters(90),  Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90),  Rotation2d.fromDegrees(180)),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(90),  Rotation2d.fromDegrees(180))
    };

    private Pose2d latestPose = new Pose2d();
    private int currentTarget = 0;

    @Override
    public void periodic() {
        // Field pose from Limelight (set LL extrinsics in web UI)
        Pose2d p = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
        if (p != null && !Double.isNaN(p.getX()) && !Double.isNaN(p.getY())) {
            latestPose = p;
        }
    }

    public Pose2d getCurrentPose() {
        return latestPose;
    }

    public Pose2d getTargetPose() {
        return TARGETS[currentTarget];
    }

    public void setTarget(int i) {
        if (i < 0) i = 0;
        if (i >= TARGETS.length) i = TARGETS.length - 1;
        currentTarget = i;
    }

    // Picks closest target to current pose
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

    // Unit direction field vector from robot to target (hypotenuse direction)
    public Translation2d getDirectionToTarget() {
        Pose2d goal = TARGETS[currentTarget];
        Translation2d d = goal.getTranslation().minus(latestPose.getTranslation());
        double n = d.getNorm();
        if (n < 1e-3) return new Translation2d();
        return new Translation2d(d.getX() / n, d.getY() / n);
    }

    // Heading to face target in field frame
    public double getHeadingToTargetDegrees() {
        Pose2d goal = TARGETS[currentTarget];
        double dx = goal.getX() - latestPose.getX();
        double dy = goal.getY() - latestPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
