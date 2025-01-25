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

  public static class LiftHardware {
   public static final int ELEVATOR_MOTOR_ID = 4;
   public static final int PIVOT_MOTOR_ID = 5;
   public static final Spark.ID OUTTAKE_MOTOR_ID = new Spark.ID("LiftHardware/Outtake", 6);
   public static final Distance SPROCKET_PITCH_RADIUS = Inches.of((1.751)/(2.0));
   public static final int INSIDE_END_EFFECTOR_BEAM_BREAK_PORT = 0;
   public static final int OUTSIDE_END_EFFECTOR_BEAM_BREAK_PORT = 1;
   public static final int ELEVATOR_HOMING_BEAM_BREAK_PORT = 2;
  }

  public static class VisionHardware {
    public static final String CAMERA_A_NAME = "Arducam_OV9782_USB_Camera_A";
    public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
      new Translation3d(-0.1016, -0.2921, 0.521),
      new Rotation3d(0.0, Math.toRadians(-26.0), Math.toRadians(+180.0))
    );
    public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_B_NAME = "Arducam_OV9782_USB_Camera_B";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
      new Translation3d(0.0254, -0.2921, 0.584),
      new Rotation3d(0.0, Math.toRadians(-25.6), 0.0)
    );
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_C_NAME = "Arducam_OV9782_USB_Camera_C";
    public static final Transform3d CAMERA_C_LOCATION = new Transform3d(
      new Translation3d(0.3, 0.0, 0.5),
      new Rotation3d(0, Math.toRadians(+15.0), Math.toRadians(180))
    );
    public static final Resolution CAMERA_C_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_C_FOV = Rotation2d.fromDegrees(79.7);
  }
}
