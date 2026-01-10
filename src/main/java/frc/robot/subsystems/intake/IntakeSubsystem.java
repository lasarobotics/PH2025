package frc.robot.subsystems.intake;

import frc.robot.lib.State;
import frc.robot.lib.StateMachine;

public class IntakeSubsystem extends StateMachine {
  private static IntakeSubsystem s_instance;

  private IntakeSubsystem() {
    super(IntakeStates.STOP);
  }

  public static IntakeSubsystem getInstance() {
    if (IntakeSubsystem.s_instance == null) {
      IntakeSubsystem.s_instance = new IntakeSubsystem();
    }
    return IntakeSubsystem.s_instance;
  }

  public enum IntakeStates implements State {
    STOP {
      @Override
      public void initialize() {
        IntakeHardware.stopIntakeMotor();
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        IntakeHardware.intake();
      }
    },
  }
}
