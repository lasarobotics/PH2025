// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class EndEffectorSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    Spark endEffectorMotor,
    DigitalInput forwardBeamBreak,
    DigitalInput reverseBeamBreak
  ) {}

  static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(50);
  static final Dimensionless REGURGITATE_MOTOR_SPEED = Percent.of(-50);
  static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(100);
  static final Dimensionless CENTER_CORAL_MOTOR_SPEED = Percent.of(-40);

  public enum EndEffectorStates implements SystemState {
    NOTHING {

      @Override
      public SystemState nextState() {
        return this;
      }

    },
    IDLE {
      @Override
      public void initialize() {
        s_endEffectorInstance.stopMotor();
      }

      @Override
      public EndEffectorStates nextState() {
        return s_endEffectorInstance.nextState;
      }
    },
    SCORE {
      @Override
      public void initialize() {
        s_endEffectorInstance.score();
      }

      @Override
      public SystemState nextState() {
        return s_endEffectorInstance.nextState;

      }
    },
    SCORE_L4 {
      @Override
      public void initialize() {
        s_endEffectorInstance.outtakeReverse();
      }

      @Override
      public SystemState nextState() {
        return s_endEffectorInstance.nextState;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        s_endEffectorInstance.intake();
        s_endEffectorInstance.m_Interrupt.enable();
      }

      @Override
      public void end(boolean interrupted) {
        s_endEffectorInstance.m_Interrupt.disable();
        s_endEffectorInstance.stopMotor();
      }

      @Override
      public SystemState nextState() {
        if(s_endEffectorInstance.forwardBeamBreakBroken()) {
          s_endEffectorInstance.nextState = HOLD;
          return HOLD;
        }
        return s_endEffectorInstance.nextState;
      }

    },
    HOLD {
      @Override
      public void execute() {
        s_endEffectorInstance.centerCoral();
      }

      @Override
      public SystemState nextState() {
        return s_endEffectorInstance.nextState;
      }

    },
    REGURGITATE {
      @Override
      public void initialize() {
        s_endEffectorInstance.outtakeReverse();
      }

      @Override
      public SystemState nextState() {
        if(s_endEffectorInstance.isEmpty()) {
          s_endEffectorInstance.nextState = IDLE;
          return IDLE;
        }
        return s_endEffectorInstance.nextState;
      }
    }
  }

  private static EndEffectorSubsystem s_endEffectorInstance;
  private final Spark m_endEffectorMotor;
  private final DigitalInput m_forwardBeamBreak;
  private final DigitalInput m_reverseBeamBreak;
  private final AsynchronousInterrupt m_Interrupt;
  private EndEffectorStates nextState;

  /**
   * Returns an instance of EndEffectorSubsystem
   *
   * @param endEffectorHardware Hardware for the system
   * @return Subsystem Instance
   */
  public static EndEffectorSubsystem getInstance(Hardware endEffectorHardware) {
    if (s_endEffectorInstance == null) {
      s_endEffectorInstance = new EndEffectorSubsystem(endEffectorHardware);
      return s_endEffectorInstance;
    } else
      return null;
  }

  /** Creates a new endEffectorSubsystem. */
  private EndEffectorSubsystem(Hardware endEffectorHardware) {
    super(EndEffectorStates.NOTHING);
    this.nextState = EndEffectorStates.NOTHING;
    this.m_endEffectorMotor = endEffectorHardware.endEffectorMotor;
    this.m_forwardBeamBreak = endEffectorHardware.forwardBeamBreak;
    this.m_reverseBeamBreak = endEffectorHardware.reverseBeamBreak;
    this.m_Interrupt = new AsynchronousInterrupt(m_forwardBeamBreak, (rising, falling) -> {
      if(falling) {
        s_endEffectorInstance.stopMotor();
      }
    });
    this.m_Interrupt.setInterruptEdges(false, true);
    m_Interrupt.disable();
  }
  /**
   * Initalizes hardware devices used by subsystem
   *
   * @return Hardware object containing all necessary devices for subsytem
   */
  public static Hardware initializeHardware() {
    Hardware endEffectorHardware = new Hardware(
      new Spark(Constants.EndEffectorHardware.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new DigitalInput(Constants.EndEffectorHardware.FORWARD_BEAM_BREAK),
      new DigitalInput(Constants.EndEffectorHardware.REVERSE_BEAM_BREAK)
      );
      return endEffectorHardware;
  }

  /**
   * Runs motor at power required for intaking
   */
  private void intake() {
    m_endEffectorMotor.set(INTAKE_MOTOR_SPEED.in(Value));
  }

  /**
   * Runs motor at power required for scoring
   */
  private void score() {
    m_endEffectorMotor.set(CENTER_CORAL_MOTOR_SPEED.in(Value));
  }

  /**
   * Runs motor at power required for scoring at L4 / for reguritating
   */
  private void outtakeReverse() {
    m_endEffectorMotor.set(SCORE_MOTOR_SPEED.in(Value));
  }

  /**
   * Stops motor
   */
  private void stopMotor() {
    m_endEffectorMotor.stopMotor();
  }

  /**
   * Checks status of forward beam break
   *
   * @return True if beam break broken, false otherwise
   */
  private boolean forwardBeamBreakBroken() {
    return !m_forwardBeamBreak.get();
  }

  /**
   * Checks status of reverse beam break
   *
   * @return True if beam break broken, false otherwise
   */
  private boolean reverseBeamBreakBroken() {
    return !m_reverseBeamBreak.get();
  }

  /**
   * Centers coral in end effector
   */
  private void centerCoral() {
    if(forwardBeamBreakBroken() && !reverseBeamBreakBroken()) {
      s_endEffectorInstance.outtakeReverse();
    } else if (reverseBeamBreakBroken() && !forwardBeamBreakBroken()) {
      s_endEffectorInstance.intake();
    } else {
      s_endEffectorInstance.stopMotor();
    }
  }

  /**
   * Checks if coral is centered in end effector
   *
   * @return True if both beam breaks are true, false otherwise
   */
  public boolean isCoralCentered() {
    return forwardBeamBreakBroken() && reverseBeamBreakBroken();
  }

  /**
   * Checks status of coral in the end effector
   *
   * @return True if end effector is empty
   */
  public boolean isEmpty() {
    return !forwardBeamBreakBroken() && !reverseBeamBreakBroken();
  }

  /**
   * Sets next state instance variable used in state machines
   * @param nextState next state to transition to
   */
  public void setState(EndEffectorStates nextState) {
    this.nextState = nextState;
  }

  public void requestScore() {
    setState(EndEffectorStates.SCORE);
  }

  public void requestScoreReverse() {
    setState(EndEffectorStates.SCORE_L4);
  }

  public void requestStop() {
    setState(EndEffectorStates.IDLE);
  }

  public void requestIntake() {
    setState(EndEffectorStates.INTAKE);
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/State", getState().toString());
    Logger.recordOutput(getName() + "/IsCoralCentered", isCoralCentered());
    Logger.recordOutput(getName() + "/forwardBeamBreak", forwardBeamBreakBroken());
    Logger.recordOutput(getName() + "/reverseBeamBreak", reverseBeamBreakBroken());
  }

  @Override
  public void close() {
    m_endEffectorMotor.close();
    m_forwardBeamBreak.close();
    m_forwardBeamBreak.close();
    s_endEffectorInstance = null;
    m_Interrupt.close();
  }
}
