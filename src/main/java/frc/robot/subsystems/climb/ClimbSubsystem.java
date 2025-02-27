// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {
  static final Dimensionless CLIMB_MOTOR_SPEED = Percent.of(100);
  public static record Hardware (
    Spark climbMotor
  ) {}

  private final Spark m_climbMotor;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Hardware ClimbHardware) {
    this.m_climbMotor = ClimbHardware.climbMotor;
  }

  /**
   * Initalizes hardware contained in Climb Subsystem
   * @return Hardware object with required hardware for Climb Subsystem
   */
  public static Hardware initializeHardware() {
    Spark climbMotor = new Spark(Constants.ClimbHardware.CLIMB_MOTOR_ID, MotorKind.NEO);
    climbMotor.setIdleMode(IdleMode.kBrake);
    Hardware climbHardware = new Hardware(
      climbMotor
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
    m_climbMotor.set(CLIMB_MOTOR_SPEED.in(Value));
  }

  /**
   * Sets the motor output for releasing
   */
  private void lowerClimber() {
    m_climbMotor.set(-CLIMB_MOTOR_SPEED.in(Value));
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
  }

  @Override
  public void close() {
    m_climbMotor.close();
  }
}
