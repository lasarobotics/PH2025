package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.lasarobotics.hardware.generic.LimitSwitch;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public final class LiftHardware {

  static final DutyCycleOut HOMING_SPEED = new DutyCycleOut(0.05);

  private static TalonFX s_elevatorMotor;
  private static TalonFX s_pivotMotor;
  private static CANcoder s_armCANcoder;
  private static LimitSwitch s_elevatorHomingBeamBreak;
  private static MotionMagicVoltage s_pivotPositionSetter;
  private static MotionMagicVoltage s_elevatorPositionSetter;

  public static void initialize() {
    s_elevatorMotor = new TalonFX(Constants.LiftHardware.ELEVATOR_MOTOR_ID.deviceID, Constants.LiftHardware.ELEVATOR_MOTOR_ID.bus.name);
    s_pivotMotor = new TalonFX(Constants.LiftHardware.PIVOT_MOTOR_ID.deviceID, Constants.LiftHardware.PIVOT_MOTOR_ID.bus.name);
    s_armCANcoder = new CANcoder(Constants.LiftHardware.ARM_CANCODER_ID.deviceID, Constants.LiftHardware.ARM_CANCODER_ID.bus.name);
    s_elevatorHomingBeamBreak = new LimitSwitch(Constants.LiftHardware.ELEVATOR_HOMING_BEAM_BREAK_PORT, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE);

    s_pivotPositionSetter = new MotionMagicVoltage(Radians.zero());
    s_elevatorPositionSetter = new MotionMagicVoltage(Radians.zero());

    // Create configurations for elevator motor
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    elevatorConfig.Feedback.SensorToMechanismRatio = 5.0;
    elevatorConfig.Feedback.RotorToSensorRatio = 1.0;
    elevatorConfig.Audio.AllowMusicDurDisable = true;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 15;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 40;
    elevatorConfig.MotionMagic.MotionMagicJerk = 0;
    elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.5;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    elevatorConfig.Slot0.kP = 18;
    elevatorConfig.Slot0.kI = 0;
    elevatorConfig.Slot0.kD = 0.065999999776482582;
    elevatorConfig.Slot0.kA = 0;
    elevatorConfig.Slot0.kV = 0.4508880257606506;
    elevatorConfig.Slot0.kG = 0.9;
    elevatorConfig.Slot0.kS = 0.099609375;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Create configurations for pivot motor
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotConfig.Feedback.RotorToSensorRatio = 72.0;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = s_armCANcoder.getDeviceID();
    pivotConfig.Audio.AllowMusicDurDisable = true;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 7.5;
    pivotConfig.MotionMagic.MotionMagicJerk = 30.0;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    pivotConfig.Slot0.kP = 100.0;
    pivotConfig.Slot0.kI = 0;
    pivotConfig.Slot0.kD = 0;
    pivotConfig.Slot0.kA = 0;
    pivotConfig.Slot0.kV = 9.162500381469727;
    pivotConfig.Slot0.kG = 0.5029296875;
    pivotConfig.Slot0.kS = 0.009765625;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    CANcoderConfiguration armCANCoderConfig = new CANcoderConfiguration();
    armCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armCANCoderConfig.MagnetSensor.MagnetOffset = 0.693115234375;
    armCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    // Apply configs for TalonFX motors
    s_elevatorMotor.getConfigurator().apply(elevatorConfig);
    s_pivotMotor.getConfigurator().apply(pivotConfig);
    s_armCANcoder.getConfigurator().apply(armCANCoderConfig);
  }

  private static String getName() {
    return LiftHardware.class.getSimpleName();
  }

  /**
   * Set arm pivot to a certain angle
   *
   * @param angle The angle you want to move the pivot
   */
  public static void setArmAngle(Angle angle) {
    Logger.recordOutput(getName() + "/targetArmAngle", angle.in(Rotations));
    s_pivotMotor.setControl(s_pivotPositionSetter.withPosition(angle));
  }

  /**
   * Set elevator to a certain height
   *
   * @param height The height you want to move the elevator to
   */
  public static void setElevatorHeight(Distance height) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);

    Logger.recordOutput(getName() + "/targetElevatorHeight", height);
    Logger.recordOutput(getName() + "/targetElevatorAngle", elevatorMoveAngle.in(Rotations));
    s_elevatorMotor.setControl(s_elevatorPositionSetter.withPosition(elevatorMoveAngle));
  }

  /**
   * Get current arm angle
   */
  public static Angle getArmAngle() {
    return s_armCANcoder.getAbsolutePosition().getValue();
  }

  /**
   * Get current arm velocity
   */
  public static AngularVelocity getArmVelocity() {
    return s_pivotMotor.getVelocity().getValue();
  }

  /**
   * Convert motor rotations to elevator A1_HEIGHT
   * @param angle in motor rotations
   * @return Distance the elevator has moved vertically
   */
  public static Distance convertToDistance(Angle angle) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Distance height = Meters.of(circumference * angle.in(Rotations));
    return height;
  }

  /**
   * Get current elevator height
   */
  public static Distance getElevatorHeight() {
    return LiftHardware.convertToDistance(s_elevatorMotor.getPosition().getValue());
  }

  /**
   * Set the elevator encoder to a given height
   */
  public static void setElevatorEncoder(Distance height) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);
    s_elevatorMotor.setPosition(elevatorMoveAngle);
  }

  /**
   * Check if elevator is at home
   *
   * @return True if elevator is home
   */
  public static boolean elevatorAtHome() {
    return elevatorHomingBeamBreak();
  }

  /**
   * Return whether the elevator is at a target height or not
   *
   * @return Boolean of if elevator is at target height
   */
  public static boolean elevatorAt(Distance targetHeight) {
    Distance currentHeight = getElevatorHeight();
    return (currentHeight.isNear(targetHeight, LiftSubsystem.ELEVATOR_TOLERANCE));
  }

  /**
   * Return whether the arm is at a target angle or not
   *
   * @return Boolean of if arm is at target angle
   */
  public static boolean armAt(Angle targetAngle) {
    Angle currentAngle = getArmAngle();
    return (currentAngle.isNear(targetAngle, LiftSubsystem.ARM_TOLERANCE));
  }

  /**
   * Return if the elevator's homing beam break is broken
   * @return Boolean for if the elevator's homing beam break is broken
   */
  public static boolean elevatorHomingBeamBreak() {
    return !s_elevatorHomingBeamBreak.getInputs().value;
  }

  /**
   * Stop the elevator motor
   */
  public static void stopElevator() {
    s_elevatorMotor.stopMotor();
  }

  /**
   * Stop the arm motor
   */
  public static void stopArm() {
    s_pivotMotor.stopMotor();
  }

  /**
   * Slowly run the elevator motor to home
   */
  public static void startHomingElevator() {
    s_elevatorMotor.setControl(HOMING_SPEED);
  }

  public static void periodic() {
    var name = LiftHardware.class.getSimpleName();
    Logger.recordOutput(name + "/homingBeamBreak", elevatorHomingBeamBreak());
    Logger.recordOutput(name + "/currentArmAngle", getArmAngle().in(Rotations));
    Logger.recordOutput(name + "/currentElevatorHeight", getElevatorHeight());
    Logger.recordOutput(name + "/currentElevatorAngle", s_elevatorMotor.getPosition().getValue().in(Rotations));
  }
}
