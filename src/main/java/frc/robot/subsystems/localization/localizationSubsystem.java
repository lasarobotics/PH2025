package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left"; // CHANGE NAME IF NEEDED
    private final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    private final NetworkTable logTable = NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    private double lastUpdate = 0.0;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double[] tcornxy = new double[8]; // [x0, y0, x1, y1, x2, y2, x3, y3]

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        // ~90 FPS -> run every ~0.011 seconds
        if (now - lastUpdate < 0.011) return;
        lastUpdate = now;

        updateFromLimelight();
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);

        // ONLY IF tv == 1
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        // LOGGING!!
        Logger.recordOutput("Localization/tv", tv);
        Logger.recordOutput("Localization/tx", tx);
        Logger.recordOutput("Localization/ty", ty);
        Logger.recordOutput("Localization/ta", ta);
        Logger.recordOutput("Localization/tcornxy", tcornxy);

        if (tv == 1) {
            // Processing values
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double[] getTcornxy() { return tcornxy; }
}
