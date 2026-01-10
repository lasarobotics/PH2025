// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.lib.State;
import frc.robot.lib.StateMachine;

public final class ClimbSubsystem extends StateMachine {

  private static ClimbSubsystem s_instance;

  private ClimbSubsystem() {
    super(ClimbStates.IDLE);
    ClimbHardware.lockClimber();
  }

  public static ClimbSubsystem getInstance() {
    if (ClimbSubsystem.s_instance == null) {
      ClimbSubsystem.s_instance = new ClimbSubsystem();
    }
    return ClimbSubsystem.s_instance;
  }

  public enum ClimbStates implements State {
    IDLE {
      @Override
      public void initialize() {
        ClimbHardware.stopClimber();
      }
    },
    MOUNT {
      @Override
      public void initialize() {
        ClimbHardware.unlockClimber();
        ClimbHardware.mount();
      }
      
      @Override
      public State nextState() {
        if((ClimbHardware.inMountPosition())) {
          return ClimbStates.IDLE;
        }
        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        ClimbHardware.climb();
      }

      @Override
      public State nextState() {
        if(ClimbHardware.inClimbPosition()) {
          return ClimbStates.HOLD;
        }
        return this;
      }
    },
    HOLD {
      @Override
      public void initialize() {
        ClimbHardware.stopClimber();
      }

      @Override
      public State nextState() {
        if(!ClimbHardware.inClimbPosition()) {
          return ClimbStates.CLIMB;
        }
        return this;
      }
    },
    STOW {
      @Override
      public void initialize() {
        ClimbHardware.stow();
      }

      @Override
      public State nextState() {
        if(ClimbHardware.inStowPosition()) {
          return ClimbStates.IDLE;
        }
        return this;
      }
    }
  }
}