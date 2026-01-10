package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public final class IntakeHardware {

  static final Dimensionless INTAKE_SPEED = Percent.of(-60);
  static final Dimensionless REVERSE_INTAKE_SPEED = Percent.of(-50);

  private static Spark s_intakeMotor;
  private static LimitSwitch s_firstBeamBreak;
  private static LimitSwitch s_secondBeamBreak;

  public static void initialize() {
    s_intakeMotor = new Spark(
        Constants.IntakeHardware.INTAKE_MOTOR_ID,
        MotorKind.NEO
    );
    s_firstBeamBreak = new LimitSwitch(
        Constants.IntakeHardware.FIRST_INTAKE_BEAM_BREAK,
        Constants.Frequencies.BEAM_BREAK_UPDATE_RATE
    );
    s_secondBeamBreak = new LimitSwitch(
        Constants.IntakeHardware.SECOND_INTAKE_BEAM_BREAK,
        Constants.Frequencies.BEAM_BREAK_UPDATE_RATE
    );
  }

  /**
   * Checks if coral is fully in the intake using the beam breaks
   * @return True if coral is fully in intake
   */
  public static boolean coralFullyInIntake() {
    return firstIntakeBeamBreak() && !secondIntakeBeamBreak();
  }

  /**
   * Checks if the intake is fully empty
   * @return True if coral is empty
   */
  public static boolean isEmpty() {
    return !firstIntakeBeamBreak() && !secondIntakeBeamBreak();
  }

  /**
   * Intake coral using intake motor
   */
  public static void intake() {
    s_intakeMotor.set(INTAKE_SPEED.in(Value));
  }

  /**
   * Outtakes the coral using the intake motor
   */
  public static void reverseIntake() {
    s_intakeMotor.set(INTAKE_SPEED.in(Value));
  }

  /**
   *  Stop the intake motor
   */
  public static void stopIntakeMotor() {
    s_intakeMotor.stopMotor();
  }

  /**
   * Returns if the first beam break in the intake is broken 
   * @return A boolean if the first beam break in the intake is broken
   */
  public static boolean firstIntakeBeamBreak() {
    return !s_firstBeamBreak.getInputs().value;
  }

  /**
   * Returns if the second beam break in the intake is broken
   * @return if the first beam break in the intake is broken
   */
  public static boolean secondIntakeBeamBreak() {
    return !s_secondBeamBreak.getInputs().value;
  }

  public static  void periodic() {
    var name = IntakeHardware.class.getSimpleName();
    Logger.recordOutput(name + "/firstBeamBreak", firstIntakeBeamBreak());
    Logger.recordOutput(name + "/secondBeamBreak", secondIntakeBeamBreak());
  }
}
