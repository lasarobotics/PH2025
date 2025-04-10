// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.ctre.phoenix6.hardware.TalonFX;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.Spark.SparkInputs;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LoopTimer;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {

  static final double CLIMB_SPEED = 1.0;
  static final double MOUNT_ANGLE = 0.357;
  static final double CLIMB_ANGLE = 0.12;
  static final double STOW_ANGLE = 0.08;

  public static record Hardware (
    Spark climbEncoder,
    TalonFX climbMotor
    
    
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
        if((s_climbInstance.inMountPosition())){
          s_climbInstance.setIsMounted(true);
        }
      }
      
      @Override
      public void execute() {
        if(!s_climbInstance.inMountPosition() && s_climbInstance.m_mounted == false){
          s_climbInstance.mount();
        } else {
          s_climbInstance.setIsMounted(true);
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
        s_climbInstance.setIsMounted(false);

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
  private final Spark m_climbEncoder;
  private final TalonFX m_climbMotor;
  private ClimbStates nextState;
  private boolean m_mounted;


  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem(Hardware ClimbHardware) {
    super(ClimbStates.IDLE);
    nextState =  ClimbStates.IDLE;

    this.m_climbEncoder = ClimbHardware.climbEncoder;
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
      new Spark(Constants.ClimbHardware.ENCODER_ID, MotorKind.NEO),
      new TalonFX(Constants.ClimbHardware.CLIMB_MOTOR_ID.deviceID, Constants.ClimbHardware.CLIMB_MOTOR_ID.bus.name)
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
   * Sets mounted variable
   */
  public void setIsMounted(boolean mounted) {
    this.m_mounted = mounted;
  }

  /**
   * Stow the climber so it is inside the frame perimeter
   */
  public void stow() {
    m_climbEncoder.set(CLIMB_SPEED);
  }

  public boolean inStowPosition() {
    return s_climbInstance.getInputs().absoluteEncoderPosition <= STOW_ANGLE;
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
    return this.m_climbEncoder.getInputs();
  }

  

  @Override
  public void periodic() {
    LoopTimer.addTimestamp(getName() + " Start");

    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/absoluteEncoderValue", s_climbInstance.getInputs().absoluteEncoderPosition);
    Logger.recordOutput(getName() + "/state", getState().toString());
    LoopTimer.addTimestamp(getName() + " End");
  }

  @Override
  public void close() {
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}