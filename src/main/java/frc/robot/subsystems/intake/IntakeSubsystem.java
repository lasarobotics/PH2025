package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware(
      Spark flapperMotor,
      Spark funnelMotor,
      LimitSwitch firstBeamBreak,
      LimitSwitch secondBeamBreak) {
  }

  static final Dimensionless INTAKE_SPEED = Percent.of(100);
  static final Dimensionless REVERSE_INTAKE_SPEED = Percent.of(-50);

  public enum IntakeStates implements SystemState {
    STOP {
      @Override
      public void initialize() {
        s_intakeInstance.stopIntakeMotor();
      }

      @Override
      public IntakeStates nextState() {
        if (s_requestedState == IDLE) {
          return IDLE;
        } else if (s_requestedState == INTAKE) {
          return INTAKE;
        } else if (s_requestedState == REGURGITATE) {
          return REGURGITATE;
        }
        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        s_intakeInstance.intake();
      }

      @Override
      public IntakeStates nextState() {
        if (s_requestedState == IDLE) {
          return IDLE;
        } else if (s_requestedState == INTAKE) {
          return INTAKE;
        } else if (s_requestedState == REGURGITATE) {
          return REGURGITATE;
        }
        return this;
      }
    },
    REGURGITATE {
      @Override
      public void initialize() {
        s_intakeInstance.reverseIntake();
      }

      @Override
      public IntakeStates nextState() {
        if (s_requestedState == IDLE) {
          return IDLE;
        } else if (s_requestedState == INTAKE) {
          return INTAKE;
        } else if (s_requestedState == REGURGITATE) {
          return REGURGITATE;
        }
        return this;
      }
    }
  }

  private static IntakeStates s_requestedState;
  private static IntakeSubsystem s_intakeInstance;
  private final Spark m_intakeMotor;
  private final LimitSwitch m_firstBeamBreak;
  private final LimitSwitch m_secondBeamBreak;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem(Hardware intakeHardware) {
    super(IntakeStates.STOP);
    this.m_intakeMotor = intakeHardware.intakeMotor;
    this.m_firstBeamBreak = intakeHardware.firstBeamBreak;
    this.m_secondBeamBreak = intakeHardware.secondBeamBreak;
    s_requestedState = IntakeStates.STOP;

    // Restore to factory defaults
    m_intakeMotor.restoreFactoryDefaults();

    // Set idle mode
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Get an instance of IntakeSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   * 
   * @param intakeHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static IntakeSubsystem getInstance(Hardware intakeHardware) {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem(intakeHardware);
      return s_intakeInstance;
    } else
      return null;
  }

  /**
   * Initialize hardware devices for intake subsystem
   * 
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.INTAKE_MOTOR_ID, MotorKind.NEO),
      new LimitSwitch(Constants.IntakeHardware.FIRST_INTAKE_BEAM_BREAK, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE),
      new LimitSwitch(Constants.IntakeHardware.SECOND_INTAKE_BEAM_BREAK, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE)
    );
    return intakeHardware;
  }

  /**
   * Calls the stop state in the state machine for API purposes
   */
  public void stop() {
    s_requestedState = IntakeStates.STOP;
  }

  /**
   * Calls the intake state in the state machine for API purpose
   */
  public void startIntake() {
    s_requestedState = IntakeStates.INTAKE;
  }

  /**
   * Calls the regurgitate state in the state machine for APi purposes
   */
  public void startRegurgitate() {
    s_requestedState = IntakeStates.REGURGITATE;
  }

  /**
   * Intake coral using only flapper intake motor
   */
  private void startFlapperIntake() {
    m_flapperMotor.set(FLAPPER_INTAKE_SPEED.in(Value));
  }

  /**
   * Intakes the coral using the funnel motor into the end effector
   */
  private void startFunnelIntake() {
    m_funnelMotor.set(FUNNEL_INTAKE_SPEED.in(Value));
  }

  /**
   * Outtakes the coral using the flapper motor
   */
  private void startReverseFlapperIntake() {
    m_flapperMotor.set(REVERSE_FLAPPER_INTAKE_SPEED.in(Value));
  }

  /**
   * Outtakes the coral using the funnel motor
   */
  private void startReverseFunnelIntake() {
    m_funnelMotor.set(REVERSE_FUNNEL_INTAKE_SPEED.in(Value));
  }

  /**
   * Checks if coral is fully in the intake using the beam breaks
   * 
   * @return Boolean value whether coral is fully in intake or not
   */
  public boolean coralFullyInIntake() {
    return ((m_firstBeamBreak.getInputs().value) && !(m_secondBeamBreak.getInputs().value));
  public boolean coralInIntake() {
    return ((m_firstBeamBreak.getInputs().value) && !(m_secondBeamBreak.getInputs().value));
  }

  /**
   * Checks if the intake is fully empty
   * @return True if coral is empty
   */
  public boolean isEmpty() {
    return !m_firstBeamBreak.getInputs().value && !m_secondBeamBreak.getInputs().value;
  }

  /**
   * Stop all the flapper motor
   */
  private void reverseIntake() {
    m_intakeMotor.set(INTAKE_SPEED.in(Value));
  }

  /**
   * Stop all the funnel motor
   */
  private void stopIntakeMotor() {
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/state", getState().toString());
  }

  /**
   * Closes all the motors, makes intake instance null
   */
  @Override
  public void close() {
    m_intakeMotor.close();
    s_intakeInstance = null;
  }
}

