package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.generic.LimitSwitch.SwitchPolarity;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    Spark flapperMotor,
    Spark funnelMotor,
    LimitSwitch firstBeamBreak,
    LimitSwitch secondBeamBreak
  ) {}

  static final Dimensionless FLAPPER_INTAKE_SPEED = Percent.of(100);
  static final Dimensionless FUNNEL_INTAKE_SPEED = Percent.of(100);

  public enum IntakeStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {
        s_intakeInstance.stopFlapperIntake();
        s_intakeInstance.stopFunnelIntake();
      }

      @Override
      public IntakeStates nextState() {
        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        s_intakeInstance.startFlapperIntake();
        s_intakeInstance.startFunnelIntake();
      }

      @Override
      public IntakeStates nextState() {
        return this;
      }
    },
    REGURGITATE {
      @Override
      public void initialize() {
        s_intakeInstance.startFunnelIntake();
        s_intakeInstance.stopFlapperIntake();
      }

      @Override
      public IntakeStates nextState() {
        if (s_intakeInstance.coralFullyInIntake())
          return IntakeStates.IDLE;
        return this;
      }
    }
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
   */
  private void startFlapperIntake() {
    m_flapperMotor.set(FLAPPER_INTAKE_SPEED.in(Value));
  }

  /**
   *	Intakes the coral using the funnel motor into the end effector
   */
  private void startFunnelIntake() {
    m_funnelMotor.set(FUNNEL_INTAKE_SPEED.in(Value));
  }

  /**
   * Checks if coral is fully in the intake using the beam breaks
   * @return Boolean value whether coral is fully in intake or not
   */
  public boolean coralFullyInIntake() {
    return ((m_firstBeamBreak.getInputs().value) && !(m_secondBeamBreak.getInputs().value));
  }

  /**
   *  Stop all the flapper motor
   */
  private void stopFlapperIntake() {
    m_flapperMotor.stopMotor();
  }

  /**
   *  Stop all the funnel motor
   */
  private void stopFunnelIntake() {
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
