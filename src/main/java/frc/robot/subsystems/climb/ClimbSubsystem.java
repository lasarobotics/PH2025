// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {

  static final double CLIMB_SPEED = 0.5;

  public static record Hardware (
    Spark climbMotor
  ) {}

  public enum ClimbStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {
        s_climbInstance.stopMotor();
      }

      @Override
      public ClimbStates nextState() {
        if(s_climbInstance.nextState == null || s_climbInstance.nextState == IDLE) {
          return this;
        } else {
          return s_climbInstance.nextState;
        }
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        s_climbInstance.climb();

      }

      @Override
      public ClimbStates nextState() {
        if(s_climbInstance.nextState != CLIMB) {
          return IDLE;
        }
        return this;
      }
    },
    RELEASE {
      @Override
      public void initialize() {
        s_climbInstance.release();

      }

      @Override
      public ClimbStates nextState() {
        if(s_climbInstance.nextState != CLIMB) {
          return IDLE;
        }
        return this;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private final Spark m_climbMotor;
  private ClimbStates nextState;

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem(Hardware ClimbHardware) {
    super(ClimbStates.IDLE);

    this.m_climbMotor = ClimbHardware.climbMotor;
  }

  /**
   * Stops motor
   */
  private void stopMotor() {
    m_climbMotor.stopMotor();
  }

  /**
   * Sets the motor output for climbing
   */
  private void climb() {
    m_climbMotor.set(CLIMB_SPEED);
  }

  /**
   * Sets the motor output for releasing
   */
  private void release() {
    m_climbMotor.set(-CLIMB_SPEED);
  }

  /**
   * Initalizes hardware contained in Climb Subsystem
   * @return Hardware object with required hardware for Climb Subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climbHardware = new Hardware(
      new Spark(Constants.ClimbHardware.CLIMB_MOTOR_ID, MotorKind.NEO)
    );

    return climbHardware;
  }

  /**
   * Gets new instance of Climb Subsystem
   * @param ClimbHardware contains hardware for climb subsystem
   * @return ClimbSubsystem object
   */
  public static ClimbSubsystem getInstance(Hardware ClimbHardware) {
    if(s_climbInstance == null){
      s_climbInstance = new ClimbSubsystem(ClimbHardware);
      return s_climbInstance;
    } else return null;
  }

  /**
   * Sets next state instance variable to CLIMB state
   */
  public void climbState() {
    this.nextState = ClimbStates.CLIMB;
  }

  /**
   * Sets next state instance variable to RELEASE state
   */
  public void releaseState() {
    this.nextState = ClimbStates.RELEASE;
  }

  /**
   * Sets next state instance variable to IDLE state
   */
  public void idleState() {
    this.nextState = ClimbStates.IDLE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {
    m_climbMotor.close();
    s_climbInstance = null;
  }
}
