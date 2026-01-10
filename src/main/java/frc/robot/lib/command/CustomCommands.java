package frc.robot.lib.command;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.Command;

public class CustomCommands {

  /**
   * Constructs a command that runs an action once and finishes.
   *
   * @param action the action to run
   * @return the command
   */
  public static Command runOnce(Runnable action) {
    return new FreeFunctionalCommand(action, () -> {}, interrupted -> {}, () -> true);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted.
   *
   * @param action the action to run
   * @return the command
   */
  public static Command run(Runnable action) {
    return new FreeFunctionalCommand(() -> {}, action, interrupted -> {}, () -> false);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted.
   *
   * @param start the action to run on start
   * @param end the action to run on interrupt
   * @return the command
   */
  public static Command startEnd(Runnable start, Runnable end) {
    requireNonNullParam(end, "end", "Command.runEnd");
    return new FreeFunctionalCommand(start, () -> {}, interrupted -> end.run(), () -> false);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action.
   *
   * @param run the action to run every iteration
   * @param end the action to run on interrupt
   * @return the command
   */
  public static Command runEnd(Runnable run, Runnable end) {
    requireNonNullParam(end, "end", "Command.runEnd");
    return new FreeFunctionalCommand(
        () -> {}, run, interrupted -> end.run(), () -> false);
  }

  /**
   * Constructs a command that runs an action once, and then runs an action every iteration until
   * interrupted.
   *
   * @param start the action to run on start
   * @param run the action to run every iteration
   * @return the command
   */
  public static Command startRun(Runnable start, Runnable run) {
    return new FreeFunctionalCommand(start, run, interrupted -> {}, () -> false);
  }
    
}
