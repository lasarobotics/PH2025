package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

public class LoopTimer {

  private static int counter = 0;
  private static long baseTime = 0;

  public static void resetTimer() {
    LoopTimer.counter = 0;
    LoopTimer.baseTime = RobotController.getFPGATime();
    Logger.recordOutput("LoopTimer/OverrunThreshold", 20.0);
  }

  public static void addTimestamp(String label) {
    long current_time = RobotController.getFPGATime();
    Logger.recordOutput("LoopTimer/" + counter + " - " + label, (current_time - baseTime) / 1000.0);
    LoopTimer.counter++;
  }
}
