
// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.lib;

import org.littletonrobotics.junction.Logger;

/** State machine subsystem */
public abstract class StateMachine {
  private State m_currentState;

  /**
   * Create a state machine
   * @param initialState Starting state for state machine
   */
  public StateMachine(State initialState) {
    this.setState(initialState);
  }

  /**
   * Set system state
   */
  public void setState(State state) {
    if (m_currentState != null) {
        m_currentState.end(state);
    }
    m_currentState = state;
    m_currentState.initialize();
  }

  public State getState() {
    return m_currentState;
  }

  /**
   * Gets the subsystem name of this State.
   * @return State name
   */
  public String getName() {
    return this.getClass().getSimpleName();
  }

  /**
   * Method to be called every loop.
   */
  public void periodic() {
    State nextState;
    while (!m_currentState.equals(nextState = m_currentState.nextState())) {
        this.setState(nextState);
    }
    m_currentState.execute();
    Logger.recordOutput(getName() + "/State", getState().getName());
  }
}

