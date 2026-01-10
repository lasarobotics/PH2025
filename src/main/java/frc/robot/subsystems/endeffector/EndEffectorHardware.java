package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public final class EndEffectorHardware {

  static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100);
  static final Dimensionless REGURGITATE_MOTOR_SPEED = Percent.of(-50);
  static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(100);
  static final Dimensionless CENTER_CORAL_MOTOR_SPEED = Percent.of(-10);
  static final Dimensionless DESCORE_ALGAE_MOTOR_SPEED = Percent.of(10);

  private static Spark s_endEffectorMotor;
  private static DigitalInput s_forwardBeamBreak;
  private static DigitalInput s_reverseBeamBreak;

  public static void initialize() {
    s_endEffectorMotor = new Spark(
      Constants.EndEffectorHardware.OUTTAKE_MOTOR_ID,
      MotorKind.NEO_550
    );
    s_forwardBeamBreak = new DigitalInput(
      Constants.EndEffectorHardware.FORWARD_BEAM_BREAK
    );
    s_reverseBeamBreak = new DigitalInput(
      Constants.EndEffectorHardware.REVERSE_BEAM_BREAK
    );
  }

  /**
   * Runs motor at power required for intaking
   */
  public static void intake() {
    s_endEffectorMotor.set(INTAKE_MOTOR_SPEED.in(Value));
  }

  /**
   * Runs motor at power required for scoring
   */
  public static void score() {
    s_endEffectorMotor.set(SCORE_MOTOR_SPEED.in(Value));
  }

  /**
   * If the coral is not fully in the end effector, center it (if it is sticking out in the reverse direction)
   */
  public static void centerReverse() {
    s_endEffectorMotor.set(-CENTER_CORAL_MOTOR_SPEED.in(Value));
  }

  /**
   * If the coral is not fully in the end effector, center it (if it is sticking out in the forward direction)
   */
  public static void centerForward() {
    s_endEffectorMotor.set(CENTER_CORAL_MOTOR_SPEED.in(Value));
  }

  /**
   * Runs motor at power required for scoring at L4 / for reguritating
   */
  public static void scoreReverse() {
    s_endEffectorMotor.set(-SCORE_MOTOR_SPEED.in(Value));
  }

  /**
   * Stops motor
   */
  public static void stopMotor() {
    s_endEffectorMotor.stopMotor();
  }

  /**
   * Checks status of forward beam break
   *
   * @return True if beam break broken, false otherwise
   */
  public static boolean forwardBeamBreakBroken() {
    return !s_forwardBeamBreak.get();
  }

  /**
   * Checks status of reverse beam break
   *
   * @return True if beam break broken, false otherwise
   */
  public static boolean reverseBeamBreakBroken() {
    return !s_reverseBeamBreak.get();
  }

  /**
   * Checks if coral is centered in end effector
   *
   * @return True if both beam breaks are true, false otherwise
   */
  public static boolean isCoralCentered() {
    return forwardBeamBreakBroken() && reverseBeamBreakBroken();
  }

  /**
   * Checks status of coral in the end effector
   *
   * @return True if end effector is empty
   */
  public static boolean isEmpty() {
    return !forwardBeamBreakBroken() && !reverseBeamBreakBroken();
  }

  public static void periodic() {
    var name = EndEffectorHardware.class.getSimpleName();
    Logger.recordOutput(name + "/IsCoralCentered", isCoralCentered());
    Logger.recordOutput(name + "/forwardBeamBreak", forwardBeamBreakBroken());
    Logger.recordOutput(name + "/reverseBeamBreak", reverseBeamBreakBroken());
  }
}