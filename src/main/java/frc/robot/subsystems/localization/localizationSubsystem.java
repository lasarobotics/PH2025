package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class localizationSubsystem extends SubsystemBase {

    private final String limelightName = "limelight-left"; // match the camera name in Limelight UI
    private final NetworkTable llTable =
        NetworkTableInstance.getDefault().getTable(limelightName);
    private final NetworkTable logTable =
        NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    private int failedCycles = 0;
    private double lastUpdate = 0.0;

    private double[] botpose = new double[7]; // X,Y,Z,Roll,Pitch,Yaw,Latency
    private int tagCount = 0;
    private double tagDistance = 0.0;
    private double tid = -1;

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < 0.02) return; // 50 Hz
        lastUpdate = now;
        updateFromLimelight();
    }

    private void updateFromLimelight() {
        try {
            double tv = llTable.getEntry("tv").getDouble(0); // valid target flag
            double[] pose = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
            double tagID = llTable.getEntry("tid").getDouble(-1);
            double[] t2d = llTable.getEntry("t2d").getDoubleArray(new double[0]);

            if (tv < 1 || pose.length < 6) {
                failedCycles++;
                Logger.recordOutput("Localization/Status", "NO TARGET");
                Logger.recordOutput("Localization/TagCount", 0);
                if (failedCycles >= 10) Logger.recordOutput("Localization/Status", "NOT RESPONDING");
                return;
            }

            failedCycles = 0;
            botpose = pose;
            tid = tagID;
            tagCount = (t2d.length >= 2) ? (int) t2d[1] : 1;
            tagDistance = (pose.length >= 7) ? pose[6] : 0.0;

            // Pose format: [X, Y, Z, Roll, Pitch, Yaw, Latency, TagCount, TagSpan, AvgDist, AvgArea]
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];
            double roll = botpose[3];
            double pitch = botpose[4];
            double yaw = botpose[5];

            Logger.recordOutput("Localization/Status", "OK");
            Logger.recordOutput("Localization/TagID", tid);
            Logger.recordOutput("Localization/TagCount", tagCount);
            Logger.recordOutput("Localization/RobotPose/X", x);
            Logger.recordOutput("Localization/RobotPose/Y", y);
            Logger.recordOutput("Localization/RobotPose/Z", z);
            Logger.recordOutput("Localization/RobotPose/Rotation", new double[]{roll, pitch, yaw});
            Logger.recordOutput("Localization/TagDistance", tagDistance);

            String formatted = String.format(
                "Pose (Field - Blue):%n" +
                " X: %.3f m%n" +
                " Y: %.3f m%n" +
                " Z: %.3f m%n" +
                " Roll: %.2f° Pitch: %.2f° Yaw: %.2f°%n" +
                " Tag ID: %.0f | Tags Seen: %d%n" +
                "--------------------------------------%n",
                x, y, z, roll, pitch, yaw, tid, tagCount
            );
            Logger.recordOutput("Localization/PoseString", formatted);

            // Also publish to NetworkTables for debugging
            logTable.getEntry("pose").setDoubleArray(botpose);
            logTable.getEntry("tid").setDouble(tid);
            logTable.getEntry("tagCount").setDouble(tagCount);
            logTable.getEntry("lastUpdateTime").setDouble(Timer.getFPGATimestamp());

        } catch (Exception e) {
            failedCycles++;
            Logger.recordOutput("Localization/Status", "EXCEPTION");
            Logger.recordOutput("Localization/failedCycles", failedCycles);
            Logger.recordOutput("Localization/ErrorMessage", e.getMessage() == null ? "unknown" : e.getMessage());
        }
    }

    public double[] getBotPose() { return botpose; }
    public double getTagID() { return tid; }
    public int getTagCount() { return tagCount; }
    public double getTagDistance() { return tagDistance; }
}
