package frc.robot.subsystems.intake;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.generic.LimitSwitch.SwitchPolarity;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    Spark frontIntakeMotor,
    Spark backIntakeMotor,
    LimitSwitch frontBeamBreak,
    LimitSwitch backBeamBreak
  ) {}

  public enum IntakeStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {

      }
      @Override
      public IntakeStates nextState() {
        return this;
      }
    };
  }

  private static IntakeSubsystem s_intakeInstance;
  private final Spark m_frontIntakeMotor;
  private final Spark m_backIntakeMotor;
  private final LimitSwitch m_frontBeamBreak;
  private final LimitSwitch m_backBeamBreak;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem(Hardware intakeHardware) {
    super(IntakeStates.IDLE);
    this.m_frontIntakeMotor = intakeHardware.frontIntakeMotor;
    this.m_backIntakeMotor = intakeHardware.backIntakeMotor;
    this.m_frontBeamBreak = intakeHardware.frontBeamBreak;
    this.m_backBeamBreak = intakeHardware.backBeamBreak;

    // Restore to factory defaults
    m_frontIntakeMotor.restoreFactoryDefaults();
    m_backIntakeMotor.restoreFactoryDefaults();

    // Set idle mode
    m_frontIntakeMotor.setIdleMode(IdleMode.kBrake);
    m_backIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Get an instance of IntakeSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   * @param intakeHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static IntakeSubsystem getInstance(Hardware intakeHardware) {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem(intakeHardware);
      return s_intakeInstance;
    } else return null;
  }

  /**
   * Initialize hardware devices for intake subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.FRONT_INTAKE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.IntakeHardware.BACK_INTAKE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new LimitSwitch(Constants.IntakeHardware.FRONT_INTAKE_BEAM_BREAK, SwitchPolarity.NORMALLY_OPEN, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE),
      new LimitSwitch(Constants.IntakeHardware.BACK_INTAKE_BEAM_BREAK, SwitchPolarity.NORMALLY_OPEN, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE)
    );
    return intakeHardware;
  }

  /**
   * Intake coral using only front intake motor
   * @param dutyCycleOutput Duty cycle to set motor to
   */
  public void frontMotorIntake(double dutyCycleOutput) {
    m_frontIntakeMotor.set(dutyCycleOutput);
  }

  /**
   *	Intakes the coral using the back motors into the end effector
   *	@param dutyCycleOutput Duty cycle to set motor to
   */
  public void backMotorIntake(double dutyCycleOutput) {
    m_backIntakeMotor.set(dutyCycleOutput);
  }

  /**
   * Checks if coral is fully in the intake using the beam breaks
   * @return Boolean value whether coral is fully in intake or not
   */
  public boolean coralFullyInIntake() {
    return ((m_frontBeamBreak.getInputs().value) && m_backBeamBreak.getInputs().value);
  }

  /**
   * Closes all the motors, makes intake instance null
   */
  public void close() {
    m_frontIntakeMotor.close();
    m_backIntakeMotor.close();
    s_intakeInstance = null;
  }
}
