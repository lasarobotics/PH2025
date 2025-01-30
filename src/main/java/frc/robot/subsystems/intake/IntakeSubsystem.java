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
    Spark flapperMotor,
    Spark funnelMotor,
    LimitSwitch firstBeamBreak,
    LimitSwitch secondBeamBreak
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
  private final Spark m_flapperMotor;
  private final Spark m_funnelMotor;
  private final LimitSwitch m_firstBeamBreak;
  private final LimitSwitch m_secondBeamBreak;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem(Hardware intakeHardware) {
    super(IntakeStates.IDLE);
    this.m_flapperMotor = intakeHardware.flapperMotor;
    this.m_funnelMotor = intakeHardware.funnelMotor;
    this.m_firstBeamBreak = intakeHardware.firstBeamBreak;
    this.m_secondBeamBreak = intakeHardware.secondBeamBreak;

    // Restore to factory defaults
    m_flapperMotor.restoreFactoryDefaults();
    m_funnelMotor.restoreFactoryDefaults();

    // Set idle mode
    m_flapperMotor.setIdleMode(IdleMode.kBrake);
    m_funnelMotor.setIdleMode(IdleMode.kBrake);
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
      new Spark(Constants.IntakeHardware.FLAPPER_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.IntakeHardware.FUNNEL_MOTOR_ID, MotorKind.NEO_VORTEX),
      new LimitSwitch(Constants.IntakeHardware.FIRST_INTAKE_BEAM_BREAK, SwitchPolarity.NORMALLY_OPEN, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE),
      new LimitSwitch(Constants.IntakeHardware.SECOND_INTAKE_BEAM_BREAK, SwitchPolarity.NORMALLY_OPEN, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE)
    );
    return intakeHardware;
  }

  /**
   * Intake coral using only flapper intake motor
   * @param dutyCycleOutput Duty cycle to set motor to
   */
  private void flapperMotorIntake(double dutyCycleOutput) {
    m_flapperMotor.set(dutyCycleOutput);
  }

  /**
   *	Intakes the coral using the funnel motor into the end effector
   *	@param dutyCycleOutput Duty cycle to set motor to
   */
  private void funnelMotorIntake(double dutyCycleOutput) {
    m_funnelMotor.set(dutyCycleOutput);
  }

  /**
   * Checks if coral is fully in the intake using the beam breaks
   * @return Boolean value whether coral is fully in intake or not
   */
  public boolean coralFullyInIntake() {
    return ((m_firstBeamBreak.getInputs().value) && !(m_secondBeamBreak.getInputs().value));
  }

  /**
   * Checks if the first beam break is broken or not
   * @return A boolean to check if the first beam break is broken or not
   */
  public boolean firstBeamBreakStatus() {
    return ((m_firstBeamBreak.getInputs().value));
  }

  /**
   *  Stops all the motors
   */
  private void stop() {
    m_flapperMotor.stopMotor();
    m_funnelMotor.stopMotor();
  }

  /**
   * Closes all the motors, makes intake instance null
   */
  public void close() {
    m_flapperMotor.close();
    m_funnelMotor.close();
    s_intakeInstance = null;
  }
}
