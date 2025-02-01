package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.lasarobotics.hardware.ctre.PhoenixCANBus;
import org.lasarobotics.hardware.ctre.TalonFX;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.vision.AprilTagCamera.Resolution;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {
  public static class Field {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Pose2d SOURCE_INTAKE_POINT = new Pose2d(
      new Translation2d(Meters.of(1.209), Meters.of(1.1)),
      Rotation2d.fromDegrees(-145.305)
    );
  }

  public static class Frequencies {
    public static final Frequency TALON_UPDATE_RATE = Hertz.of(50);
    public static final Frequency BEAM_BREAK_UPDATE_RATE = Hertz.of(50);
  }

  public static class Drive {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(5); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1); // TODO measure

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_RATE.in(RadiansPerSecond),
      MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)
    );
    public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_SPEED.in(MetersPerSecond),
      MAX_ACCELERATION.in(MetersPerSecondPerSecond)
    );

    public static final double TURN_P = 0.01;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
}

  public static class EndEffector {
    public static final double INTAKE_MOTOR_SPEED = 0.5;
    public static final double REGURGITATE_MOTOR_SPEED = -0.5;
    public static final double SCORE_MOTOR_SPEED = 1.0;
    public static final double CENTER_CORAL_MOTOR_SPEED = -0.4;
  }

  public static class LiftHardware {
   public static final TalonFX.ID ELEVATOR_MOTOR_ID = new TalonFX.ID("LiftHardware/Elevator", PhoenixCANBus.RIO, 4);
   public static final TalonFX.ID PIVOT_MOTOR_ID = new TalonFX.ID("LiftHardware/Pivot", PhoenixCANBus.RIO, 5);
   public static final Distance SPROCKET_PITCH_RADIUS = Inches.of((1.751)/(2.0));
   public static final LimitSwitch.ID ELEVATOR_HOMING_BEAM_BREAK_PORT = new LimitSwitch.ID("LiftHardware/HomingSwitch", 0);
  }

  public static class IntakeHardware {
    public static final Spark.ID FLAPPER_MOTOR_ID = new Spark.ID("IntakeHardware/FlapperIntakeMotor", 7);
    public static final Spark.ID FUNNEL_MOTOR_ID = new Spark.ID("IntakeHardware/FrontIntakeMotor", 8);
    public static final LimitSwitch.ID FIRST_INTAKE_BEAM_BREAK = new LimitSwitch.ID("IntakeHardware/FirstIntakeBeamBreak", 1);
    public static final LimitSwitch.ID SECOND_INTAKE_BEAM_BREAK = new LimitSwitch.ID("IntakeHardware/SecondIntakeBeamBreak", 2);
  }

  public static class EndEffectorHardware {
    public static final Spark.ID OUTTAKE_MOTOR_ID =  new Spark.ID("endEffecterMotor", 7);
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
