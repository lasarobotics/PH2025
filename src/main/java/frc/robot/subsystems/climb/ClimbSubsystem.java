// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.SparkInputs;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {

  static final double CLIMB_SPEED = 1.0;
  static final double MOUNT_ANGLE = 0.946;
  static final double CLIMB_ANGLE = 0.692;

  public static record Hardware (
    Spark climbMotor
    
  ) {}

  public enum ClimbStates implements SystemState {
    NOTHING {
      @Override
      public ClimbStates nextState() {
        return ClimbStates.NOTHING;
      }
    },
    IDLE {
      @Override
      public void initialize() {
        s_climbInstance.stopMotor();
      }

      @Override
      public ClimbStates nextState() {
        return s_climbInstance.nextState;
      }
    },
    MOUNT {
      @Override
      public void initialize() {
        s_climbInstance.mount();
      }
      
      @Override
      public void execute() {
        if(!s_climbInstance.inMountPosition() && s_climbInstance.m_mounted == true){
          s_climbInstance.mount();
        } else {
          s_climbInstance.m_mounted = true;
          s_climbInstance.stopMotor();
        }
      }

      @Override
      public ClimbStates nextState() {
        return s_climbInstance.nextState;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        s_climbInstance.climb();
        s_climbInstance.m_mounted = false;

      }

      @Override
      public void execute() {
        if(!s_climbInstance.inClimbPosition()){
          s_climbInstance.climb();
        } else {
          s_climbInstance.stopMotor();
        }
      }

      @Override
      public ClimbStates nextState() {
        return s_climbInstance.nextState;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private final Spark m_climbMotor;
  private ClimbStates nextState;
  private boolean m_mounted;


  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem(Hardware ClimbHardware) {
    super(ClimbStates.IDLE);
    nextState =  ClimbStates.IDLE;

    this.m_climbMotor = ClimbHardware.climbMotor;
    this.m_mounted = false;
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
  private void mount() {
    m_climbMotor.set(-CLIMB_SPEED);
  }

  /**
   * Runs climber
   * @return Command to run the climber motors
   */
  public Command raiseClimberCommand() {
    return runEnd(() -> climb(), () -> stopMotor());
  }

  /**
   * Runs climber backward
   * @return Command to run the climber motors
   */
  public Command lowerClimberCommand() {
    return runEnd(() -> mount(), () -> stopMotor());
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
  public void mountState() {
    this.nextState = ClimbStates.MOUNT;
  }

  /**
   * Returns whether or not climber is in mount position
   */
  public boolean inMountPosition() {
    if(s_climbInstance.getInputs().absoluteEncoderPosition > MOUNT_ANGLE) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Returns whether or not climber is in its retracted fully climbed position
   */
  public boolean inClimbPosition() {
    if(s_climbInstance.getInputs().absoluteEncoderPosition <= CLIMB_ANGLE) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Sets next state instance variable to IDLE state
   */
  public void idleState() {
    this.nextState = ClimbStates.IDLE;
  }

  /**
   * Returns true if climb subsystem is in mount state, false if climb subsystem is in climb state or rest state
   */
  public boolean isMounting() {
    if(this.nextState == ClimbStates.MOUNT) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * gets spark inputs
   */
  public SparkInputs getInputs() {
    return this.m_climbMotor.getInputs();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/absoluteEncoderValue", s_climbInstance.getInputs().absoluteEncoderPosition);
    Logger.recordOutput(getName() + "/state", getState().toString());
  }

  @Override
  public void close() {
    m_climbMotor.close();
    s_climbInstance = null;
  }
}