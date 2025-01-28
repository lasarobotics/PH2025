// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffecter;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class EndEffecterSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
    Spark EndEffecterMotor,
    DigitalInput insideEndEffectorBeamBreak,
    DigitalInput outsideEndEffectorBeamBreak
  ) {}

  private Spark m_EndEffecterMotor;
  private DigitalInput m_insideEndEffectorBeamBreak;
  private DigitalInput m_outsideEndEffectorBeamBreak;

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
  }
  /** Creates a new EndEffecterSubsystem. */
  public EndEffecterSubsystem(Hardware EndEffecterHardware){
    super(EndEffecterStates.IDLE);
    this.m_EndEffecterMotor = EndEffecterHardware.EndEffecterMotor;
    this.m_insideEndEffectorBeamBreak = EndEffecterHardware.insideEndEffectorBeamBreak;
    this.m_outsideEndEffectorBeamBreak = EndEffecterHardware.outsideEndEffectorBeamBreak;
  }


  public static Hardware initializeHardware(){
    Hardware EndEffecterHardware = new Hardware(
      new Spark(Constants.EndEffecterHardware.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new DigitalInput(Constants.EndEffecterHardware.INSIDE_END_EFFECTOR_BEAM_BREAK_PORT),
      new DigitalInput(Constants.EndEffecterHardware.OUTSIDE_END_EFFECTOR_BEAM_BREAK_PORT));

      return EndEffecterHardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void close() {
    m_EndEffecterMotor.close();
    m_insideEndEffectorBeamBreak.close();
    m_outsideEndEffectorBeamBreak.close();
  }
}
