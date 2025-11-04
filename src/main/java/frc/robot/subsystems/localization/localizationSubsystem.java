package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left"; // match your Limelight name
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
        // run ~90 FPS (every ~11ms)
        if (now - lastUpdate < 0.011) return;
        lastUpdate = now;

        updateFromLimelight();
        calculate3DPosition();
        calculateTagArea();
        calculateDiagonalsAndMidpoint();
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);

        // read latest 2D info
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        // always log, even if tv == 0
        Logger.recordOutput("Localization/tv", tv);
        Logger.recordOutput("Localization/tx", tx);
        Logger.recordOutput("Localization/ty", ty);
        Logger.recordOutput("Localization/ta", ta);

        // individual tcornxy elements
        if (tcornxy.length >= 8) {
            Logger.recordOutput("Localization/x0", tcornxy[0]);
            Logger.recordOutput("Localization/y0", tcornxy[1]);
            Logger.recordOutput("Localization/x1", tcornxy[2]);
            Logger.recordOutput("Localization/y1", tcornxy[3]);
            Logger.recordOutput("Localization/x2", tcornxy[4]);
            Logger.recordOutput("Localization/y2", tcornxy[5]);
            Logger.recordOutput("Localization/x3", tcornxy[6]);
            Logger.recordOutput("Localization/y3", tcornxy[7]);
        }

        // if valid target, publish to NT
        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }
    }

    // Computes side lengths between corners
    public void calculate3DPosition() {
        if (tcornxy.length < 8) return;

        // using corner layout:
        // (3)----(2)
        //  |      |
        // (0)----(1)

        double TLtoBL = Math.abs(tcornxy[7] - tcornxy[1]); // y3 - y0
        double TRtoBR = Math.abs(tcornxy[5] - tcornxy[3]); // y2 - y1
        double TLtoTR = Math.abs(tcornxy[6] - tcornxy[4]); // x3 - x2
        double BLtoBR = Math.abs(tcornxy[0] - tcornxy[2]); // x0 - x1

        Logger.recordOutput("Localization/TLtoBL", TLtoBL);
        Logger.recordOutput("Localization/TRtoBR", TRtoBR);
        Logger.recordOutput("Localization/TLtoTR", TLtoTR);
        Logger.recordOutput("Localization/BLtoBR", BLtoBR);
    }

    // Calculates approximate area of the detected tag on the image
    public void calculateTagArea() {
        if (tcornxy.length < 8) return;

        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        // Heights
        double hLeft = Math.abs(y3 - y0);
        double hRight = Math.abs(y2 - y1);
        double avgHeight = (hLeft + hRight) / 2.0;

        // Widths
        double wTop = Math.abs(x2 - x3);
        double wBottom = Math.abs(x1 - x0);
        double avgWidth = (wTop + wBottom) / 2.0;

        // Area in pixel units
        double area = avgWidth * avgHeight;

        // Log all values
        Logger.recordOutput("Localization/hLeft", hLeft);
        Logger.recordOutput("Localization/hRight", hRight);
        Logger.recordOutput("Localization/avgHeight", avgHeight);
        Logger.recordOutput("Localization/wTop", wTop);
        Logger.recordOutput("Localization/wBottom", wBottom);
        Logger.recordOutput("Localization/avgWidth", avgWidth);
        Logger.recordOutput("Localization/TagArea", area);
    }

    // Calculate diagonals and midpoint of tag corners
    public void calculateDiagonalsAndMidpoint() {
        if (tcornxy.length < 8) return;

        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        // Diagonals
        double diag1 = Math.sqrt(Math.pow(x2 - x0, 2) + Math.pow(y2 - y0, 2));
        double diag2 = Math.sqrt(Math.pow(x3 - x1, 2) + Math.pow(y3 - y1, 2));

        // Midpoint of all four corners
        double midX = (x0 + x1 + x2 + x3) / 4.0;
        double midY = (y0 + y1 + y2 + y3) / 4.0;

        // Log results
        Logger.recordOutput("Localization/Diagonal1", diag1);
        Logger.recordOutput("Localization/Diagonal2", diag2);
        Logger.recordOutput("Localization/MidpointX", midX);
        Logger.recordOutput("Localization/MidpointY", midY);

        // Optional console print for quick testing
        System.out.printf("Diagonals: %.2f, %.2f | Midpoint: (%.2f, %.2f)%n", diag1, diag2, midX, midY);
    }

    // Accessors
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double[] getTcornxy() { return tcornxy; }
}
