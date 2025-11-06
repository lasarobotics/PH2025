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

    // ---- Vision → Motion planning parameters (logic-only) ----

    // If you want “±10 pixels” centering, set DEG_PER_PIXEL and we derive deg tolerance.
    // For LL @ 320x240 & ~54° HFOV, ~0.16875 deg/pixel; adjust for your cam/pipeline.
    private static final double DEG_PER_PIXEL = 0.16875; // tune if you want pixel-based centering
    private static final double PIXEL_TOLERANCE = 10.0;  // requested ±10 (if using pixel logic)
    private static final boolean USE_PIXEL_TOL_FOR_TX = false; // set true to use pixel mapping
    private static final double TX_TOLERANCE_DEG = USE_PIXEL_TOL_FOR_TX ? (PIXEL_TOLERANCE * DEG_PER_PIXEL) : 1.0;

    // Stop 1 inch from tag
    private static final double DESIRED_RANGE_M = 0.0254;

    // Calibration: provide TA (area) measured at these distances (feet) facing the tag
    // Fill these with your measured values (TA is Limelight “ta” at each distance).
    private static final double TA_AT_1FT  =  /* TODO */ 0.0;
    private static final double TA_AT_3FT  =  /* TODO */ 0.0;
    private static final double TA_AT_5FT  =  /* TODO */ 0.0;
    private static final double TA_AT_7FT  =  /* TODO */ 0.0;
    private static final double TA_AT_10FT =  /* TODO */ 0.0;

    // Small clamps/safety
    private static final double MIN_TA = 1e-6;      // avoid divide-by-zero
    private static final double MAX_REASONABLE_METERS = 15.0; // field-wide sanity cap

    // Outputs for your drive code (logic only, no actuation here)
    public static final class Plan {
        public double forwardErrorMeters; // + means move forward toward tag, - back away
        public double strafeErrorMeters;  // + means strafe left, - strafe right (field/camera-aligned)
        public double headingErrorDeg;    // rotate to make heading 0° (as requested)
        public boolean atRange;           // ~1 inch from tag
        public boolean centered;          // tx within tolerance
        public boolean facing;            // heading ~ 0°
        public String recommendation;     // "MOVE", "STRAFE", "ROTATE", or "ALIGNED"
    }

    private final Plan currentPlan = new Plan();

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < 0.011) return; // ~90 FPS
        lastUpdate = now;

        updateFromLimelight();

        // compute geometric logs if corners valid (optional)
        if (isCornersValid()) {
            calculate3DPosition();
            calculateTagArea();
            calculateDiagonalsAndMidpoint();
        } else {
            Logger.recordOutput("Localization/Status", "No valid corners");
        }

        // Update the alignment plan from vision (uses tx, ta, and robot heading you pass in)
        // You MUST call setRobotHeadingDeg(...) from elsewhere before this, or use a gyro reading here.
        computePlanFromVision();
        logPlan(currentPlan);
    }

    // ---- Limelight IO ----
    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        // Console for quick checks
        System.out.println("tv=" + tv + " tx=" + tx + " ty=" + ty + " ta=" + ta);
        System.out.println("tcornxy: " + Arrays.toString(tcornxy));

        // Publish to NT only if valid target
        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }
    }

    private boolean isCornersValid() {
        if (tcornxy == null || tcornxy.length < 8) return false;
        for (double v : tcornxy) if (Math.abs(v) > 1e-3) return true;
        return false;
    }

    // ---- Geometry logs (unchanged from your version) ----
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

    // ---- Robot heading source (you provide this from gyro elsewhere) ----
    private double robotHeadingDeg = 0.0;
    public void setRobotHeadingDeg(double headingDeg) { this.robotHeadingDeg = headingDeg; }

    // ---- TA → distance model and alignment plan (logic only) ----

    private static double feetToMeters(double ft) { return ft * 0.3048; }

    private double estimateDistanceMetersFromTA(double taNow) {
        // Build ks from provided calibration points (d = k / sqrt(ta))
        // Use median k for robustness
        double[] ds_ft  = new double[] {1, 3, 5, 7, 10};
        double[] tas     = new double[] {TA_AT_1FT, TA_AT_3FT, TA_AT_5FT, TA_AT_7FT, TA_AT_10FT};
        double[] ks      = new double[5];
        int n = 0;
        for (int i = 0; i < 5; i++) {
            double ta_i = Math.max(tas[i], MIN_TA);
            if (ta_i > MIN_TA) {
                ks[n++] = feetToMeters(ds_ft[i]) * Math.sqrt(ta_i);
            }
        }
        if (n == 0) return MAX_REASONABLE_METERS; // no calibration yet

        Arrays.sort(ks, 0, n);
        double kMed = (n % 2 == 1) ? ks[n/2] : 0.5 * (ks[n/2 - 1] + ks[n/2]);

        double taSafe = Math.max(taNow, MIN_TA);
        double d = kMed / Math.sqrt(taSafe);
        if (Double.isNaN(d) || Double.isInfinite(d)) d = MAX_REASONABLE_METERS;
        return Math.min(d, MAX_REASONABLE_METERS);
    }

    private static double normalizeDeg(double deg) {
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    private void computePlanFromVision() {
        // 1) Estimate current forward range from tag via TA calibration
        double rangeM = estimateDistanceMetersFromTA(ta);

        // 2) Compute strafe error from tx: lateral ≈ range * tan(tx)
        double txRad = Math.toRadians(tx);
        double strafeM = rangeM * Math.tan(txRad); // +left, -right (camera frame)

        // 3) Heading error to 0° (as requested)
        double headingErrDeg = normalizeDeg(0.0 - robotHeadingDeg);

        // 4) Forward error to stop at 1 inch
        double forwardErrM = rangeM - DESIRED_RANGE_M; // + means move forward

        // 5) Tolerances
        boolean centered = Math.abs(tx) <= TX_TOLERANCE_DEG;
        boolean atRange  = Math.abs(forwardErrM) <= 0.01; // ±1 cm around 1 inch
        boolean facing   = Math.abs(headingErrDeg) <= 2.0;

        // 6) Recommendation ordering
        String rec;
        if (!atRange)       rec = "MOVE";    // prioritize range first
        else if (!facing)   rec = "ROTATE";  // then heading
        else if (!centered) rec = "STRAFE";  // then crosshair centering
        else                rec = "ALIGNED";

        // Populate plan
        currentPlan.forwardErrorMeters = forwardErrM;
        currentPlan.strafeErrorMeters  = strafeM;
        currentPlan.headingErrorDeg    = headingErrDeg;
        currentPlan.atRange            = atRange;
        currentPlan.centered           = centered;
        currentPlan.facing             = facing;
        currentPlan.recommendation     = rec;
    }

    private void logPlan(Plan p) {
        Logger.recordOutput("Align/forwardError_m", p.forwardErrorMeters);
        Logger.recordOutput("Align/strafeError_m",  p.strafeErrorMeters);
        Logger.recordOutput("Align/headingError_deg", p.headingErrorDeg);
        Logger.recordOutput("Align/atRange",  p.atRange);
        Logger.recordOutput("Align/centered", p.centered);
        Logger.recordOutput("Align/facing",   p.facing);
        Logger.recordOutput("Align/recommendation", p.recommendation);
    }

    // ---- Accessors your drive code can consume later ----
    public Plan getPlan() { return currentPlan; }

    // Existing accessors
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double[] getTcornxy() { return tcornxy; }
}
