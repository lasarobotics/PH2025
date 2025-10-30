package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.LimelightHelpers;

public class localizationSubsystem extends SubsystemBase {

    private Notifier notifier;
    private final String limelightName = "limelight-left"; //CHANGE NAME TO REAL NAME
    private int failedCycles = 0;

    public double currentX0 = 0.0, currentY0 = 0.0;
    public double currentX1 = 0.0, currentY1 = 0.0;
    public double currentX2 = 0.0, currentY2 = 0.0;
    public double currentX3 = 0.0, currentY3 = 0.0;

    public localizationSubsystem() {
        notifier = new Notifier(this::updateDetections);
        notifier.startPeriodic(0.1);
        System.out.println("[LocalizationSubsystem] Started Reading");
    }

    private void updateDetections() {
        try {
            LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
            System.out.println("------ LIMELIGHT RAW DETECTIONS ------");
            if (detections == null) {
                failedCycles++;
                System.err.printf("NULL DATA", limelightName, failedCycles);
                if (failedCycles >= 10) {
                    System.err.printf("NOT RESPONDING", limelightName);
                }
                return;
            }
            if (detections.length == 0) {
                failedCycles++;
                System.out.printf("NO DETECTIONS", failedCycles);
                return;
            }
            failedCycles = 0;
            LimelightHelpers.RawDetection d = detections[0];
            currentX0 = d.corner0_X;
            currentY0 = d.corner0_Y;
            currentX1 = d.corner1_X;
            currentY1 = d.corner1_Y;
            currentX2 = d.corner2_X;
            currentY2 = d.corner2_Y;
            currentX3 = d.corner3_X;
            currentY3 = d.corner3_Y;
            System.out.printf(
                "Detection #0:%n" +
                " Corner0: (%.2f, %.2f)%n" +
                " Corner1: (%.2f, %.2f)%n" +
                " Corner2: (%.2f, %.2f)%n" +
                " Corner3: (%.2f, %.2f)%n" +
                "--------------------------------------%n",
                currentX0, currentY0,
                currentX1, currentY1,
                currentX2, currentY2,
                currentX3, currentY3
            );
        } catch (Exception e) {
            failedCycles++;
            System.err.printf("Exception reading Limelight '%s' (cycle %d): %s%n", limelightName, failedCycles, e.getMessage());
        }
        System.out.println();
    }

    public double[] getCornersX() {
        return new double[]{ currentX0, currentX1, currentX2, currentX3 };
    }

    public double[] getCornersY() {
        return new double[]{ currentY0, currentY1, currentY2, currentY3 };
    }
}
