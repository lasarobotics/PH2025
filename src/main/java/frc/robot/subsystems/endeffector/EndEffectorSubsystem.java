// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.lib.State;
import frc.robot.lib.StateMachine;
import frc.robot.subsystems.lift.LiftHardware;

public final class EndEffectorSubsystem extends StateMachine {
  private static EndEffectorSubsystem s_instance;

  private EndEffectorSubsystem() {
    super(EndEffectorStates.HOLD);
  }

  public static EndEffectorSubsystem getInstance() {
    if (EndEffectorSubsystem.s_instance == null) {
      EndEffectorSubsystem.s_instance = new EndEffectorSubsystem();
    }
    return EndEffectorSubsystem.s_instance;
  }

  public enum EndEffectorStates implements State {
    IDLE {
      @Override
      public void initialize() {
        EndEffectorHardware.stopMotor();
      }
    },
    SCORE_L1_L2 {
      @Override
      public void initialize() {
        EndEffectorHardware.score();
      }
    },
    SCORE_L3_L4 {
      @Override
      public void initialize() {
        EndEffectorHardware.scoreReverse();
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        EndEffectorHardware.intake();
      }

      @Override
      public void end(State nexState) {
        EndEffectorHardware.stopMotor();
      }

      @Override
      public State nextState() {
        if(
          EndEffectorHardware.reverseBeamBreakBroken() ||
          EndEffectorHardware.forwardBeamBreakBroken()
        ) {
          return HOLD;
        }
        return this;
      }

    },
    HOLD {
      @Override
      public void execute() {
        if (LiftHardware.getArmVelocity().gte(RotationsPerSecond.of(0.85))) {
          EndEffectorHardware.centerReverse();
        } else if(
          EndEffectorHardware.forwardBeamBreakBroken() &&
          !EndEffectorHardware.reverseBeamBreakBroken()
        ) {
          EndEffectorHardware.centerForward();
        } else if (
          EndEffectorHardware.reverseBeamBreakBroken() &&
          !EndEffectorHardware.forwardBeamBreakBroken()
        ) {
          EndEffectorHardware.centerReverse();
        } else {
          EndEffectorHardware.stopMotor();
        }
      }
    }
  }
}
