// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffecter;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import frc.robot.Constants;

public class EndEffecterSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
    Spark EndEffecterMotor
  ) {}

  public static enum USER_INPUT {
    SCORE
  }

  public enum EndEffecterStates implements SystemState{
    IDLE {
      @Override
      public void initialize() {

      }

      @Override
      public EndEffecterStates nextState() {
        return this;
      }
    },
    SCORE {
      @Override
      public void initialize() {
        s_EndEffecterInstance.disableForwardLimitSwitch();
        s_EndEffecterInstance.disableReverseLimitSwitch();
        s_EndEffecterInstance.setMotorPower(1.0);
      }
      
      @Override
      public SystemState nextState(){
        if (getUserInput() == USER_INPUT.SCORE) {
          return IDLE;
        }
        return this;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        s_EndEffecterInstance.enableForwardLimitSwitch();
        s_EndEffecterInstance.setMotorPower(1.0);
      }

      @Override
      public void execute() {
        s_EndEffecterInstance.centerCoral();

        if(s_EndEffecterInstance.isCoralCentered()) {
          nextState();
        }
      }

      @Override
      public SystemState nextState() {
        USER_INPUT userInput = getUserInput();
        return IDLE;
      }

    },
    REGURGITATE {
      @Override
      public void initialize() {
        s_EndEffecterInstance.setMotorPower(-1.0);
      }

      @Override
      public SystemState nextState(){
        return IDLE;
      }
    }

  }

  
  private static EndEffecterSubsystem s_EndEffecterInstance;
  
  private Spark m_EndEffecterMotor;
  private LimitSwitch m_insideEndEffectorBeamBreak;
  private LimitSwitch m_outsideEndEffectorBeamBreak;

  public static EndEffecterSubsystem getInstance(Hardware EndEffecterHardware){
    if(s_EndEffecterInstance == null){
      s_EndEffecterInstance = new EndEffecterSubsystem(EndEffecterHardware);
      return s_EndEffecterInstance;
    } else return null;
  }

    /** Creates a new EndEffecterSubsystem. */
  public EndEffecterSubsystem(Hardware EndEffecterHardware){
    super(EndEffecterStates.IDLE);
    this.m_EndEffecterMotor = EndEffecterHardware.EndEffecterMotor;

    enableForwardLimitSwitch();
    enableReverseLimitSwitch();
  }


  public static Hardware initializeHardware(){
    Hardware EndEffecterHardware = new Hardware(
      new Spark(Constants.EndEffecterHardware.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX));

      return EndEffecterHardware;
  }

  private void setMotorPower(double power) {
    m_EndEffecterMotor.set(power);
  }

  private void stopMotor() {
    m_EndEffecterMotor.stopMotor();
  }

  private void enableForwardLimitSwitch(){
    m_EndEffecterMotor.enableForwardLimitSwitch();
  }

  private void enableReverseLimitSwitch(){
    m_EndEffecterMotor.enableReverseLimitSwitch();
  }

  private void disableForwardLimitSwitch() {
    m_EndEffecterMotor.disableForwardLimitSwitch();
  }

  private void disableReverseLimitSwitch() {
    m_EndEffecterMotor.disableReverseLimitSwitch();
  }

  private boolean forwardBeamBreakStatus(){
    return m_EndEffecterMotor.getInputs().forwardLimitSwitch;
  }

  private boolean reverseBeamBreakStatus(){
    return m_EndEffecterMotor.getInputs().reverseLimitSwitch;
  }

  private static USER_INPUT getUserInput(){
    return USER_INPUT.SCORE;
  }

  private void centerCoral() {
    if(forwardBeamBreakStatus()) {
      m_EndEffecterMotor.enableReverseLimitSwitch();
      m_EndEffecterMotor.set(-0.4);
    }
    if(reverseBeamBreakStatus()) {
      s_EndEffecterInstance.stopMotor();
    }
  }

  private boolean isCoralCentered() {
    return forwardBeamBreakStatus() && reverseBeamBreakStatus();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void close() {
    m_EndEffecterMotor.close();
    m_insideEndEffectorBeamBreak.close();
    m_outsideEndEffectorBeamBreak.close();
    s_EndEffecterInstance = null;
  }
}
