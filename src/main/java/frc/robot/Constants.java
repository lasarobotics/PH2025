package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class Drive {
        public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(5); // TODO measure
        public static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75); // TODO measure
        public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1); // TODO measure

        public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_RATE.in(RadiansPerSecond),
            MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

        public static final double TURN_P = 0.01;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
    }
}
