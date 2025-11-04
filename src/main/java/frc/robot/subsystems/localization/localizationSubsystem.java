package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left"; // must match Limelight name in UI
    private final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    private final NetworkTable logTable = NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    private double lastUpdate = 0.0;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double[] tcornxy = new double[8]; // [x0,y0,x1,y1,x2,y2,x3,y3]

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        // Run ~90 FPS (~11 ms per update)
        if (now - lastUpdate < 0.011)
            return;
        lastUpdate = now;

        updateFromLimelight();

        // only compute geometry if valid corners
        if (isCornersValid()) {
            calculate3DPosition();
            calculateTagArea();
            calculateDiagonalsAndMidpoint();
        } else {
            Logger.recordOutput("Localization/Status", "No valid corners");
        }
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        // Always log
        // Logger.recordOutput("Localization/tv", tv);
        // Logger.recordOutput("Localization/tx", tx);
        // Logger.recordOutput("Localization/ty", ty);
        // Logger.recordOutput("Localization/ta", ta);

        // Individual corner points
        if (tcornxy.length >= 8) {
            // Logger.recordOutput("Localization/x0", tcornxy[0]);
            // Logger.recordOutput("Localization/y0", tcornxy[1]);
            // Logger.recordOutput("Localization/x1", tcornxy[2]);
            // Logger.recordOutput("Localization/y1", tcornxy[3]);
            // Logger.recordOutput("Localization/x2", tcornxy[4]);
            // Logger.recordOutput("Localization/y2", tcornxy[5]);
            // Logger.recordOutput("Localization/x3", tcornxy[6]);
            // Logger.recordOutput("Localization/y3", tcornxy[7]);
        }

        // Publish to NetworkTables only if valid target
        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }

        // Console debug
        System.out.println("tv=" + tv + " tx=" + tx + " ty=" + ty + " ta=" + ta);
        System.out.println("tcornxy: " + Arrays.toString(tcornxy));
    }

    /** Checks whether corner data is valid (not all zeros and length = 8) */
    private boolean isCornersValid() {
        if (tcornxy == null || tcornxy.length < 8)
            return false;
        for (double v : tcornxy) {
            if (Math.abs(v) > 1e-3)
                return true; // some non-zero value
        }
        return false;
    }

    /** Vertical & horizontal side lengths */
    private void calculate3DPosition() {
        double y0 = tcornxy[1], y1 = tcornxy[3], y2 = tcornxy[5], y3 = tcornxy[7];
        double x0 = tcornxy[0], x1 = tcornxy[2], x2 = tcornxy[4], x3 = tcornxy[6];

        double hLeft = Math.abs(y3 - y0);
        double hRight = Math.abs(y2 - y1);
        double wTop = Math.abs(x2 - x3);
        double wBottom = Math.abs(x1 - x0);

        Logger.recordOutput("Localization/hLeft", hLeft);
        Logger.recordOutput("Localization/hRight", hRight);
        Logger.recordOutput("Localization/wTop", wTop);
        Logger.recordOutput("Localization/wBottom", wBottom);

        // System.out.printf("Heights: L=%.2f R=%.2f Widths: T=%.2f B=%.2f%n", hLeft,
        // hRight, wTop, wBottom);
    }

    /** Approximates tag area in pixel² */
    private void calculateTagArea() {
        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        double hLeft = Math.abs(y3 - y0);
        double hRight = Math.abs(y2 - y1);
        double wTop = Math.abs(x2 - x3);
        double wBottom = Math.abs(x1 - x0);

        double avgHeight = (hLeft + hRight) / 2.0;
        double avgWidth = (wTop + wBottom) / 2.0;
        double area = avgWidth * avgHeight;

        Logger.recordOutput("Localization/avgHeight", avgHeight);
        Logger.recordOutput("Localization/avgWidth", avgWidth);
        Logger.recordOutput("Localization/TagArea", area);

        // System.out.printf("Tag pixel area: %.2f (avgW=%.2f avgH=%.2f)%n", area,
        // avgWidth, avgHeight);
    }

    /** Calculates diagonals and midpoint */
    private void calculateDiagonalsAndMidpoint() {
        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        double diag1 = Math.sqrt(Math.pow(x2 - x0, 2) + Math.pow(y2 - y0, 2));
        double diag2 = Math.sqrt(Math.pow(x3 - x1, 2) + Math.pow(y3 - y1, 2));
        double midX = (x0 + x1 + x2 + x3) / 4.0;
        double midY = (y0 + y1 + y2 + y3) / 4.0;

        Logger.recordOutput("Localization/Diagonal1", diag1);
        Logger.recordOutput("Localization/Diagonal2", diag2);
        Logger.recordOutput("Localization/MidpointX", midX);
        Logger.recordOutput("Localization/MidpointY", midY);

        // System.out.printf("Diag1=%.2f Diag2=%.2f Midpoint=(%.2f, %.2f)%n", diag1,
        // diag2, midX, midY);
    }

    // Accessors
    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public double[] getTcornxy() {
        return tcornxy;
    }
}
