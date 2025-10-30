package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class localizationSubsystem extends SubsystemBase {

    private Notifier notifier;
    private final String limelightName = "limelight-left";
    private int failedCycles = 0;

    private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    public double currentX0 = 0.0, currentY0 = 0.0;
    public double currentX1 = 0.0, currentY1 = 0.0;
    public double currentX2 = 0.0, currentY2 = 0.0;
    public double currentX3 = 0.0, currentY3 = 0.0;

    public localizationSubsystem() {
        notifier = new Notifier(this::updateDetections);
        notifier.startPeriodic(0.02);
        System.out.println("[LocalizationSubsystem] Started Reading");
    }

    @Override
    public void periodic() {
        updateDetections();
    }

    private void updateDetections() {
        try {
            LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);

            if (detections == null) {
                failedCycles++;
                Logger.recordOutput("Localization/Status", "NULL DATA");
                if (failedCycles >= 10) Logger.recordOutput("Localization/Status", "NOT RESPONDING");
                return;
            }

            if (detections.length == 0) {
                failedCycles++;
                Logger.recordOutput("Localization/Status", "NO DETECTIONS");
                Logger.recordOutput("Localization/DetectionCount", 0);
                return;
            }

            failedCycles = 0;
            LimelightHelpers.RawDetection d = detections[0];

            currentX0 = d.corner0_X; currentY0 = d.corner0_Y;
            currentX1 = d.corner1_X; currentY1 = d.corner1_Y;
            currentX2 = d.corner2_X; currentY2 = d.corner2_Y;
            currentX3 = d.corner3_X; currentY3 = d.corner3_Y;

            Logger.recordOutput("Localization/Status", "OK");
            Logger.recordOutput("Localization/DetectionCount", detections.length);
            Logger.recordOutput("Localization/Corners/X", new double[]{currentX0, currentX1, currentX2, currentX3});
            Logger.recordOutput("Localization/Corners/Y", new double[]{currentY0, currentY1, currentY2, currentY3});

            String detectionString = String.format(
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

            Logger.recordOutput("Localization/DetectionString", detectionString);

            table.getEntry("corner0").setDoubleArray(new double[]{currentX0, currentY0});
            table.getEntry("corner1").setDoubleArray(new double[]{currentX1, currentY1});
            table.getEntry("corner2").setDoubleArray(new double[]{currentX2, currentY2});
            table.getEntry("corner3").setDoubleArray(new double[]{currentX3, currentY3});
            table.getEntry("lastUpdateTime").setDouble(System.currentTimeMillis() / 1000.0);

        } catch (Exception e) {
            failedCycles++;
            Logger.recordOutput("Localization/Status", "EXCEPTION");
            Logger.recordOutput("Localization/ErrorMessage", e.getMessage());
        }
    }

    public double[] getCornersX() {
        return new double[]{currentX0, currentX1, currentX2, currentX3};
    }

    public double[] getCornersY() {
        return new double[]{currentY0, currentY1, currentY2, currentY3};
    }

    public static void localizationSubsystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'localizationSubsystem'");
    }
}
