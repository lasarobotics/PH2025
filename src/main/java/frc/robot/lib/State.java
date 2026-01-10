// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.lib;

/** System state */
public interface State {
  /**
   * Initial action of state. Called once when state is initially scheduled
   */
  default void initialize() {}

  /**
   * Main body of state. Called repeatedly when state is scheduled.
   */
  default void execute() {}

  /**
   * The action to take when the state ends. Called when either the state finishes normally, or when it interrupted/canceled.
   * @param nextState The state that the state machine is transitioning to
   */
  default void end(State nextState) {}

  /**
   * Get next state based on variety of inputs. Also used to know if current state is complete.
   * @return Next state
   */
  default State nextState() {
    return this;
  };

  /**
   * Gets the subsystem name of this State.
   * @return State name
   */
  default String getName() {
    return this.getClass().getSimpleName();
  }
}
