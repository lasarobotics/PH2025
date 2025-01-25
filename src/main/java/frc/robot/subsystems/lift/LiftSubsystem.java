package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SystemState;


public class LiftSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    TalonFX elevatorMotor,
    TalonFX pivotMotor,
    Spark outtakeMotor,
    DigitalInput insideEndEffectorBeamBreak,
    DigitalInput outsideEndEffectorBeamBreak,
    DigitalInput elevatorHomingBeamBreak
  ) {}

  public enum LiftStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {

      }

      @Override
      public LiftStates nextState() {
        return this;
      }
    };
  }

  private static LiftSubsystem s_liftinstance;

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_pivotMotor;
  private final Spark m_outtakeMotor;
  private final DigitalInput m_insideEndEffectorBeamBreak;
  private final DigitalInput m_outsideEndEffectorBeamBreak;
  private final DigitalInput m_elevatorHomingBeamBreak;

  /**Creates a new LiftSubsystem. */
  private LiftSubsystem(Hardware liftHardware, SparkPIDConfig outtakeMotorPIDConfig) {
    super(LiftStates.IDLE);
    this.m_elevatorMotor = liftHardware.elevatorMotor;
    this.m_pivotMotor = liftHardware.pivotMotor;
    this.m_outtakeMotor = liftHardware.outtakeMotor;
    this.m_insideEndEffectorBeamBreak = liftHardware.insideEndEffectorBeamBreak;
    this.m_outsideEndEffectorBeamBreak = liftHardware.outsideEndEffectorBeamBreak;
    this.m_elevatorHomingBeamBreak = liftHardware.elevatorHomingBeamBreak;

    //TODO: figure out motor configuration values (gear ratios, sensors, etc)

    //Create configurations for elevator motor
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    elevatorConfig.Feedback.SensorToMechanismRatio = 1.0;
    elevatorConfig.Feedback.RotorToSensorRatio = 1.0;
    elevatorConfig.Audio.AllowMusicDurDisable = true;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 0;
    elevatorConfig.MotionMagic.MotionMagicJerk = 0;
    elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    elevatorConfig.Slot0.kP = 0;
    elevatorConfig.Slot0.kI = 0;
    elevatorConfig.Slot0.kD = 0;
    elevatorConfig.Slot0.kA = 0;
    elevatorConfig.Slot0.kV = 0;
    elevatorConfig.Slot0.kG = 0;
    elevatorConfig.Slot0.kS = 0;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;


    //Create configurations for pivot motor
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 120;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 70;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
    pivotConfig.Feedback.RotorToSensorRatio = 1.0;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Audio.AllowMusicDurDisable = true;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 0;
    pivotConfig.MotionMagic.MotionMagicJerk = 0;
    pivotConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    pivotConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    pivotConfig.Slot0.kP = 0;
    pivotConfig.Slot0.kI = 0;
    pivotConfig.Slot0.kD = 0;
    pivotConfig.Slot0.kA = 0;
    pivotConfig.Slot0.kV = 0;
    pivotConfig.Slot0.kG = 0;
    pivotConfig.Slot0.kS = 0;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    //Apply SparkFlex configs
    SparkMaxConfig outtakeConfig = new SparkMaxConfig();
    outtakeConfig.smartCurrentLimit(0); // To-do constants
    m_outtakeMotor.configure(outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Apply configs for TalonFX motors
    m_elevatorMotor.getConfigurator().apply(elevatorConfig);
    m_pivotMotor.getConfigurator().apply(pivotConfig);

  }

  /**
   * Get an instance of LiftSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   * @param LiftHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static LiftSubsystem getInstance(Hardware liftHardware, SparkPIDConfig outtakeMotorPIDConfig) {
    if (s_liftinstance == null) {
      s_liftinstance = new LiftSubsystem(liftHardware, outtakeMotorPIDConfig);
      return s_liftinstance;
    } else return null;
  }

  /**
   * Initialize hardware devices for lift subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware liftHardware = new Hardware(
      new TalonFX(Constants.Lift.ELEVATOR_MOTOR_ID),
      new TalonFX(Constants.Lift.PIVOT_MOTOR_ID),
      new Spark(Constants.Lift.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new DigitalInput(Constants.Lift.INSIDE_END_EFFECTOR_BEAM_BREAK_PORT),
      new DigitalInput(Constants.Lift.OUTSIDE_END_EFFECTOR_BEAM_BREAK_PORT),
      new DigitalInput(Constants.Lift.ELEVATOR_HOMING_BEAM_BREAK_PORT)
    );
    return liftHardware;
  }

  /**
   * Move arm pivot to a certain angle
   * @param angle The angle you want to move the pivot
   */
  private void movePivot(Angle angle) {
    m_pivotMotor.setControl(new MotionMagicVoltage(angle));
  }

  /**
   * Move elevator to a certain position
   * @param Distance The distance you want to move the elevator to
   */
  private void moveElevator(Distance distance) {
    Distance SPROCKET_RADIUS = Constants.Lift.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveangle = Rotations.of(distance.in(Meters) / circumference);
    m_elevatorMotor.setControl(new MotionMagicVoltage(elevatorMoveangle));
  }

  /**
   * Run the scoring outtake to score coral on the reef
   * @param dutyCycleOutput The dutycycle output to set the motor at for outtake
   */
  private void runScoringMech(double dutyCycleOutput) {
    m_outtakeMotor.set(dutyCycleOutput);
  }

  /**
   * Zero the elevator encoder
   */
  public void homeElevator() {
    m_elevatorMotor.setPosition(Degrees.of(0));
  }

  public boolean elevatorAtHome() {
    return m_elevatorHomingBeamBreak.get();
  }

  public boolean isCoralReady() {
    return m_insideEndEffectorBeamBreak.get() && m_outsideEndEffectorBeamBreak.get();
  }


  public void close() {
    m_elevatorMotor.close();
    m_pivotMotor.close();
    m_outtakeMotor.close();
    s_liftinstance = null;
  }
}
