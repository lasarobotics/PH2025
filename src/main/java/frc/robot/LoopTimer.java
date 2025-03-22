package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

public class LoopTimer {

  private static int couter = 0;
  private static long baseTime = 0;

  public static void resetTimer() {
    LoopTimer.couter = 0;
    LoopTimer.baseTime = RobotController.getFPGATime();
    Logger.recordOutput("LoopTimer/OverrunThreshold", 20 * 1000);
  }

  public static void addTimestamp(String label) {
    long current_time = RobotController.getFPGATime();
    Logger.recordOutput("LoopTimer/" + couter + " - " + label, current_time - baseTime);
    LoopTimer.couter++;
  }
}
