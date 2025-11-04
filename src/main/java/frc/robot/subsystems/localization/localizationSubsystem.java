// x0 bottom left cord, x1 bottom right cord, x2 top right cord, x3 top left cord
package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left"; // CHANGE IF NEEDED
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
        // 90 fps
        if (now - lastUpdate < 0.011) return;
        lastUpdate = now;

        updateFromLimelight();
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);

        // latest stuff
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);

        // Logging
        Logger.recordOutput("Localization/tv", tv);
        Logger.recordOutput("Localization/tx", tx);
        Logger.recordOutput("Localization/ty", ty);
        Logger.recordOutput("Localization/ta", ta);

        // tcornxy logging
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

        // check if valid target works
        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }
    }
    
    public void calculate3DPosition() {
        double TLtoBL = Math.abs(tcornxy[7] - tcornxy[1]);
        double TRtoBR = Math.abs(tcornxy[5] - tcornxy[3]);
        double TLtoTR = Math.abs(tcornxy[6] - tcornxy[4]);
        double BLtoBR = Math.abs(tcornxy[0] - tcornxy[2]);
        Logger.recordOutput("Localization/TLtoBL", TLtoBL);
        Logger.recordOutput("Localization/TRtoBR", TRtoBR);
        Logger.recordOutput("Localization/TLtoTR", TLtoTR);
        Logger.recordOutput("Localization/BLtoBR", BLtoBR);
    }

    // Acces3ors
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public double[] getTcornxy() { return tcornxy; }
}
