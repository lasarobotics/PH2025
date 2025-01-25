package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.vision.AprilTagCamera.Resolution;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {
 public static class Drive {
  public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75); // TODO measure this value
 }

 public static class Lift {
    public static final int ELEVATOR_MOTOR_ID = 4;
    public static final int PIVOT_MOTOR_ID = 5;
    public static final Spark.ID OUTTAKE_MOTOR_ID = new Spark.ID("LiftHardware/Outtake", 6);
    public static final Distance SPROCKET_PITCH_RADIUS = Inches.of((1.751)/(2.0));
    public static final int INSIDE_END_EFFECTOR_BEAM_BREAK_PORT = 0;
    public static final int OUTSIDE_END_EFFECTOR_BEAM_BREAK_PORT = 1;
    public static final int ELEVATOR_HOMING_BEAM_BREAK_PORT = 2;
 }
}
