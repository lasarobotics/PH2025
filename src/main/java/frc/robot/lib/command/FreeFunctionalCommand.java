package frc.robot.lib.command;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;

public class FreeFunctionalCommand extends Command {
  private final Runnable m_onInit;
  private final Runnable m_onExecute;
  private final Consumer<Boolean> m_onEnd;
  private final BooleanSupplier m_isFinished;

  /**
   * Creates a new FunctionalCommand.
   *
   * @param onInit the function to run on command initialization
   * @param onExecute the function to run on command execution
   * @param onEnd the function to run on command end
   * @param isFinished the function that determines whether the command has finished
   */
  @SuppressWarnings("this-escape")
  public FreeFunctionalCommand(
      Runnable onInit,
      Runnable onExecute,
      Consumer<Boolean> onEnd,
      BooleanSupplier isFinished) {
    m_onInit = requireNonNullParam(onInit, "onInit", "FunctionalCommand");
    m_onExecute = requireNonNullParam(onExecute, "onExecute", "FunctionalCommand");
    m_onEnd = requireNonNullParam(onEnd, "onEnd", "FunctionalCommand");
    m_isFinished = requireNonNullParam(isFinished, "isFinished", "FunctionalCommand");
  }

  @Override
  public void initialize() {
    m_onInit.run();
  }

  @Override
  public void execute() {
    m_onExecute.run();
  }

  @Override
  public void end(boolean interrupted) {
    m_onEnd.accept(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_isFinished.getAsBoolean();
  }
}
