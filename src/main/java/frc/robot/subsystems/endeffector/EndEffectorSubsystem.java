// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import frc.robot.Constants;

public class EndEffectorSubsystem extends StateMachine implements AutoCloseable {

  public static record Hardware (
    Spark endEffectorMotor
  ) {}

  public enum endEffectorStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {
        s_endEffectorInstance.stopMotor();
      }

      @Override
      public endEffectorStates nextState() {
        if(s_endEffectorInstance.nextState == null || s_endEffectorInstance.nextState == IDLE) {
          return this;
        } else {
          return s_endEffectorInstance.nextState;
        }
      }
    },
    SCORE {
      @Override
      public void initialize() {
        s_endEffectorInstance.disableForwardLimitSwitch();
        s_endEffectorInstance.disableReverseLimitSwitch();
        s_endEffectorInstance.score();
      }
      
      @Override
      public SystemState nextState() {
        if(s_endEffectorInstance.nextState != SCORE) {
          s_endEffectorInstance.enableForwardLimitSwitch();
          s_endEffectorInstance.enableReverseLimitSwitch();

          return IDLE;
        }
        return this;

      }
    },
    INTAKE {
      @Override
      public void initialize() {
        s_endEffectorInstance.enableForwardLimitSwitch();
        s_endEffectorInstance.intake();
      }

      @Override
      public void execute() {
        s_endEffectorInstance.centerCoral();
      }

      @Override
      public SystemState nextState() {
        if(s_endEffectorInstance.nextState != INTAKE) {
          s_endEffectorInstance.enableForwardLimitSwitch();
          s_endEffectorInstance.enableReverseLimitSwitch();
  
          return IDLE;
        }
        return this;
      }

    },
    SCORE_L4 {
      @Override
      public void initialize() {
        s_endEffectorInstance.disableForwardLimitSwitch();
        s_endEffectorInstance.disableReverseLimitSwitch();
        s_endEffectorInstance.scoreL4();
      }
      
      @Override
      public SystemState nextState() {
        if(s_endEffectorInstance.nextState != SCORE) {
          s_endEffectorInstance.enableForwardLimitSwitch();
          s_endEffectorInstance.enableReverseLimitSwitch();
  
          return IDLE;
        }
        return this;
      }
    },
    REGURGITATE {
      @Override
      public void initialize() {
        s_endEffectorInstance.regurgitate();   
      }

      @Override
      public SystemState nextState(){
        if(s_endEffectorInstance.nextState != REGURGITATE) {  
          return IDLE;
        }
        return this;
      }
    }

  }

  
  private static EndEffectorSubsystem s_endEffectorInstance;
  private Spark m_endEffectorMotor;
  private LimitSwitch m_insideEndEffectorBeamBreak;
  private LimitSwitch m_outsideEndEffectorBeamBreak;
  private endEffectorStates nextState;

  /**
   * Returns an instance of EndEffecterSubsystem
   * 
   * @param endEffectorHardware Hardware for the system
   * @return Subsystem Instance
   */
  public static EndEffectorSubsystem getInstance(Hardware endEffectorHardware) {
    if(s_endEffectorInstance == null){
      s_endEffectorInstance = new EndEffectorSubsystem(endEffectorHardware);
      return s_endEffectorInstance;
    } else return null;
  }

    /** Creates a new endEffectorSubsystem. */
  public EndEffectorSubsystem(Hardware endEffectorHardware) {
    super(endEffectorStates.IDLE);
    this.m_endEffectorMotor = endEffectorHardware.endEffectorMotor;

    enableForwardLimitSwitch();
    enableReverseLimitSwitch();
  }

  /**
   * Initalizes hardware devices used by subsystem
   * @return Hardware object containing all necessary devices for subsytem
   */
  public static Hardware initializeHardware(){
    Hardware endEffectorHardware = new Hardware(
      new Spark(Constants.EndEffectorHardware.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX));

      return endEffectorHardware;
  }
  /**
   * Sets motor power
   * @param power double ranging from -1.0 to 1.0
   */
  private void setMotorPower(double power) {
    m_endEffectorMotor.set(power);
  }

  /**
   * Runs motor at power required for intaking
   */
  private void intake() {
    m_endEffectorMotor.set(Constants.EndEffector.INTAKE_MOTOR_SPEED);
  }

  /**
   * Runs motor at power required for scoring
   */
  private void score() {
    m_endEffectorMotor.set(Constants.EndEffector.SCORE_MOTOR_SPEED);
  }

  /**
   * Runs motor at power required for scoring at L4
   */
  private void scoreL4() {
    m_endEffectorMotor.set(-Constants.EndEffector.SCORE_MOTOR_SPEED);
  }

  /**
   * Regurgitates Coral back into lift
   */
  private void regurgitate() {
    m_endEffectorMotor.set(Constants.EndEffector.REGURGITATE_MOTOR_SPEED);
  }

  /**
   * Stops motor
   */
  private void stopMotor() {
    m_endEffectorMotor.stopMotor();
  }

  /**
   * Enables forward beam break
   */
  private void enableForwardLimitSwitch(){
    m_endEffectorMotor.enableForwardLimitSwitch();
  }

  /**
   * Enables reverse beam break
   */
  private void enableReverseLimitSwitch(){
    m_endEffectorMotor.enableReverseLimitSwitch();
  }

  /**
   * Disables forward beam break 
   */
  private void disableForwardLimitSwitch() {
    m_endEffectorMotor.disableForwardLimitSwitch();
  }

  /** 
   * Disables reverse beam break
   */
  private void disableReverseLimitSwitch() {
    m_endEffectorMotor.disableReverseLimitSwitch();
  }

  /**
   * Checks Status of forward beam break
   * @return true if beam break broken, false otherwise
   */
  private boolean forwardBeamBreakStatus(){
    return m_endEffectorMotor.getInputs().forwardLimitSwitch;
  }

  /**
   * Checks Status of reverse beam break
   * @return true if beam break broken, false otherwise
   */
  private boolean reverseBeamBreakStatus(){
    return m_endEffectorMotor.getInputs().reverseLimitSwitch;
  }

  /**
   * Centers Coral in end effecter
   */
  private void centerCoral() {
    if(forwardBeamBreakStatus()) {
      m_endEffectorMotor.enableReverseLimitSwitch();
      m_endEffectorMotor.set(-0.4);
    }
    if(reverseBeamBreakStatus()) {
      s_endEffectorInstance.stopMotor();
    }
  }

  /**
   * Checks if coral is centered in end effecter
   * @return true if both beam breaks are true, false otherwise
   */
  public boolean isCoralCentered() {
    return forwardBeamBreakStatus() && reverseBeamBreakStatus();
  }

  /**
   * Sets next state instance variable used in state machines
   * @param nextState next state to transition to
   */
  public void setNextState(endEffectorStates nextState) {
    this.nextState = nextState;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void close() {
    m_endEffectorMotor.close();
    m_insideEndEffectorBeamBreak.close();
    m_outsideEndEffectorBeamBreak.close();
    s_endEffectorInstance = null;
  }
}
