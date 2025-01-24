package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.lasarobotics.hardware.revrobotics.Spark;

import edu.wpi.first.units.measure.AngularVelocity;
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
 }
}
