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
import frc.robot.LoopTimer;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    Spark intakeMotor,
    LimitSwitch firstBeamBreak,
    LimitSwitch secondBeamBreak
  ) {}

  static final Dimensionless INTAKE_SPEED = Percent.of(-60);
  static final Dimensionless REVERSE_INTAKE_SPEED = Percent.of(-50);

  public enum IntakeStates implements SystemState {
    NOTHING {

      @Override
      public SystemState nextState() {
        return this;
      }

    },
    STOP {
      @Override
      public void initialize() {
        s_intakeInstance.stopIntakeMotor();
      }

      @Override
      public IntakeStates nextState() {
        if (s_requestedState == STOP) {
          return STOP;
        }
        else if (s_requestedState == INTAKE) {
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
        if (s_requestedState == STOP) {
          return STOP;
        }
        else if (s_requestedState == INTAKE) {
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
        if (s_requestedState == STOP) {
          return STOP;
        }
        else if (s_requestedState == INTAKE) {
          return INTAKE;
        }
        else if (s_requestedState == REGURGITATE) {
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
   * Checks if coral is fully in the intake using the beam breaks
   * @return True if coral is fully in intake
   */
  public boolean coralFullyInIntake() {
    return firstIntakeBeamBreak() && !secondIntakeBeamBreak();
  }

  /**
   * Checks if the intake is fully empty
   * @return True if coral is empty
   */
  public boolean isEmpty() {
    return !firstIntakeBeamBreak() && !secondIntakeBeamBreak();
  }

  /**
   * Intake coral using intake motor
   */
  private void intake() {
    m_intakeMotor.set(INTAKE_SPEED.in(Value));
  }

  /**
   * Outtakes the coral using the intake motor
   */
  private void reverseIntake() {
    m_intakeMotor.set(INTAKE_SPEED.in(Value));
  }

  /**
   *  Stop the intake motor
   */
  private void stopIntakeMotor() {
    m_intakeMotor.stopMotor();
  }

  /**
   * Returns if the first beam break in the intake is broken 
   * @return A boolean if the first beam break in the intake is broken
   */
  public boolean firstIntakeBeamBreak() {
    return !m_firstBeamBreak.getInputs().value;
  }

  /**
   * Returns if the second beam break in the intake is broken
   * @return if the first beam break in the intake is broken
   */
  public boolean secondIntakeBeamBreak() {
    return !m_secondBeamBreak.getInputs().value;
  }

  @Override
  public void periodic() {
    LoopTimer.addTimestamp(getName() + " Start");
    super.periodic();

    Logger.recordOutput(getName() + "/state", getState().toString());
    Logger.recordOutput(getName() + "/firstBeamBreak", firstIntakeBeamBreak());
    Logger.recordOutput(getName() + "/secondBeamBreak", secondIntakeBeamBreak());
    
    LoopTimer.addTimestamp(getName() + " End");
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
