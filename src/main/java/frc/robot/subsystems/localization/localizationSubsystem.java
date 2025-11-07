package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left";
    private final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    private final NetworkTable logTable = NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    private double lastUpdate = 0.0;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double[] tcornxy = new double[8]; // [x0,y0,x1,y1,x2,y2,x3,y3]

    // ---- Vision → Motion planning parameters ----
    private static final double DEG_PER_PIXEL = 0.16875;
    private static final double PIXEL_TOLERANCE = 10.0;
    private static final boolean USE_PIXEL_TOL_FOR_TX = false;
    private static final double TX_TOLERANCE_DEG = USE_PIXEL_TOL_FOR_TX ? (PIXEL_TOLERANCE * DEG_PER_PIXEL) : 1.0;
    private static final double DESIRED_RANGE_M = 0.0254; // 1 inch

    private static final double MIN_TA = 1e-6;
    private static final double MAX_REASONABLE_METERS = 15.0;

    // ---- TA calibration ranges ----
    private double[] TA_1FT  = {29000, 31000};
    private double[] TA_3FT  = {12200, 12400};
    private double[] TA_5FT  = {6200, 6400};
    private double[] TA_7FT  = {3600, 3800};
    private double[] TA_10FT = {1900, 2100};

    // ---- Plan output ----
    public static final class Plan {
        public double forwardErrorMeters;
        public double strafeErrorMeters;
        public double headingErrorDeg;
        public boolean atRange;
        public boolean centered;
        public boolean facing;
        public String recommendation;
    }

    private final Plan currentPlan = new Plan();

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < 0.011) return; // ~90 FPS
        lastUpdate = now;

        updateFromLimelight();

        if (isCornersValid()) {
            calculate3DPosition();
            calculateTagArea();
            calculateDiagonalsAndMidpoint();
        } else {
            Logger.recordOutput("Localization/Status", "No valid corners");
        }

        computePlanFromVision();
        logPlan(currentPlan);
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }

        System.out.println("tv=" + tv + " tx=" + tx + " ty=" + ty + " ta=" + ta);
    }

    private boolean isCornersValid() {
        if (tcornxy == null || tcornxy.length < 8) return false;
        for (double v : tcornxy) if (Math.abs(v) > 1e-3) return true;
        return false;
    }

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
    }

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
    }

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
    }

    private double robotHeadingDeg = 0.0;
    public void setRobotHeadingDeg(double headingDeg) { this.robotHeadingDeg = headingDeg; }

    private static double feetToMeters(double ft) { return ft * 0.3048; }

    /** Compute distance from TA using average of min/max calibration ranges */
    private double estimateDistanceMetersFromTA(double taNow) {
        double[] ds_ft = {1, 3, 5, 7, 10};
        double[] tas = {
            avg(TA_1FT), avg(TA_3FT), avg(TA_5FT), avg(TA_7FT), avg(TA_10FT)
        };
        double[] ks = new double[5];
        int n = 0;

        for (int i = 0; i < 5; i++) {
            double ta_i = Math.max(tas[i], MIN_TA);
            if (ta_i > MIN_TA)
                ks[n++] = feetToMeters(ds_ft[i]) * Math.sqrt(ta_i);
        }
        if (n == 0) return MAX_REASONABLE_METERS;

        Arrays.sort(ks, 0, n);
        double kMed = (n % 2 == 1) ? ks[n / 2] : 0.5 * (ks[n / 2 - 1] + ks[n / 2]);
        double taSafe = Math.max(taNow, MIN_TA);
        double d = kMed / Math.sqrt(taSafe);
        if (Double.isNaN(d) || Double.isInfinite(d)) d = MAX_REASONABLE_METERS;
        return Math.min(d, MAX_REASONABLE_METERS);
    }

    private static double avg(double[] range) { return (range[0] + range[1]) / 2.0; }

    private static double normalizeDeg(double deg) {
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    private void computePlanFromVision() {
        double rangeM = estimateDistanceMetersFromTA(ta);
        double txRad = Math.toRadians(tx);
        double strafeM = rangeM * Math.tan(txRad);
        double headingErrDeg = normalizeDeg(0.0 - robotHeadingDeg);
        double forwardErrM = rangeM - DESIRED_RANGE_M;

        boolean centered = Math.abs(tx) <= TX_TOLERANCE_DEG;
        boolean atRange = Math.abs(forwardErrM) <= 0.01;
        boolean facing = Math.abs(headingErrDeg) <= 2.0;

        String rec;
        if (!atRange) rec = "MOVE";
        else if (!facing) rec = "ROTATE";
        else if (!centered) rec = "STRAFE";
        else rec = "ALIGNED";

        currentPlan.forwardErrorMeters = forwardErrM;
        currentPlan.strafeErrorMeters = strafeM;
        currentPlan.headingErrorDeg = headingErrDeg;
        currentPlan.atRange = atRange;
        currentPlan.centered = centered;
        currentPlan.facing = facing;
        currentPlan.recommendation = rec;
    }

    private void logPlan(Plan p) {
        Logger.recordOutput("Align/forwardError_m", p.forwardErrorMeters);
        Logger.recordOutput("Align/strafeError_m", p.strafeErrorMeters);
        Logger.recordOutput("Align/headingError_deg", p.headingErrorDeg);
        Logger.recordOutput("Align/atRange", p.atRange);
        Logger.recordOutput("Align/centered", p.centered);
        Logger.recordOutput("Align/facing", p.facing);
        Logger.recordOutput("Align/recommendation", p.recommendation);
    }

    // ---- Manual range setters ----
    public void setCalibrationRanges(
        double[] ft1, double[] ft3, double[] ft5, double[] ft7, double[] ft10) {
        TA_1FT = ft1.clone();
        TA_3FT = ft3.clone();
        TA_5FT = ft5.clone();
        TA_7FT = ft7.clone();
        TA_10FT = ft10.clone();

        Logger.recordOutput("Calibration/1ft_range", TA_1FT);
        Logger.recordOutput("Calibration/3ft_range", TA_3FT);
        Logger.recordOutput("Calibration/5ft_range", TA_5FT);
        Logger.recordOutput("Calibration/7ft_range", TA_7FT);
        Logger.recordOutput("Calibration/10ft_range", TA_10FT);
    }

    public Plan getPlan() { return currentPlan; }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double[] getTcornxy() { return tcornxy; }
}
