// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.ejml.equation.IntegerSequence.For;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.lift.LiftSubsystem;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {
  static final Dimensionless CLIMB_MOTOR_SPEED = Percent.of(100);
  public static record Hardware (
    Spark climbMotor,
    LimitSwitch forwardLimitSwitch,
    LimitSwitch reverseLimitSwitch
  ) {}

  private final Spark m_climbMotor;
  private final LimitSwitch m_forwardLimitSwitch;
  private final LimitSwitch m_reverseLimitSwitch;
  // private final AsynchronousInterrupt m_forwardInterrupt;
  // private final AsynchronousInterrupt m_reverseInterrupt;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Hardware ClimbHardware) {
    this.m_climbMotor = ClimbHardware.climbMotor;
    this.m_forwardLimitSwitch = ClimbHardware.forwardLimitSwitch;
    this.m_reverseLimitSwitch = ClimbHardware.reverseLimitSwitch;
    
    // m_forwardInterrupt = m_forwardLimitSwitch.bindInterrupt((rising, falling) -> {
    //   if (rising)
    //     m_climbMotor.stopMotor();
    //   }, true, false
    // );
    // m_reverseInterrupt = m_reverseLimitSwitch.bindInterrupt((rising, falling) -> {
    //   if (rising)
    //     m_climbMotor.stopMotor();
    //   }, true, false
    // );
  }

  /**
   * Initalizes hardware contained in Climb Subsystem
   * @return Hardware object with required hardware for Climb Subsystem
   */
  public static Hardware initializeHardware() {
    Spark climbMotor = new Spark(Constants.ClimbHardware.CLIMB_MOTOR_ID, MotorKind.NEO);
    climbMotor.setIdleMode(IdleMode.kBrake);
    Hardware climbHardware = new Hardware(
      climbMotor,
      new LimitSwitch(Constants.ClimbHardware.FORWARD_LIMIT_SWITCH_ID, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE),
      new LimitSwitch(Constants.ClimbHardware.REVERSE_LIMIT_SWITCH_ID, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE)
    );
    return climbHardware;
  }

  /**
   * Stops motor
   */
  private void stop() {
    m_climbMotor.stopMotor();
  }

  /**
   * Sets the motor output for climbing
   */
  private void raiseClimber() {
    // if (m_reverseLimitSwitch.getInputs().value)
      m_climbMotor.set(CLIMB_MOTOR_SPEED.in(Value));
    // else
    //   stop();
  }

  /**
   * Sets the motor output for releasing
   */
  private void lowerClimber() {
    // if (m_forwardLimitSwitch.getInputs().value)
      m_climbMotor.set(-CLIMB_MOTOR_SPEED.in(Value));
    // else
    //   stop();
  }

  /**
   * Runs climber
   * @return Command to run the climber motors
   */
  public Command raiseClimberCommand() {
    return runEnd(() -> raiseClimber(), () -> stop());
  }

  /**
   * Runs climber backward
   * @return Command to run the climber motors
   */
  public Command lowerClimberCommand() {
    return runEnd(() -> lowerClimber(), () -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Forward Limit", !m_forwardLimitSwitch.getInputs().value);
  }

  @Override
  public void close() {
    m_climbMotor.close();
  }
}
