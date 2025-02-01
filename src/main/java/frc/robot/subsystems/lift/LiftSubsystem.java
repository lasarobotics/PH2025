package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.hardware.ctre.TalonFX;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.lasarobotics.hardware.generic.LimitSwitch.SwitchPolarity;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class LiftSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    TalonFX elevatorMotor,
    TalonFX pivotMotor,
    LimitSwitch elevatorHomingBeamBreak
  ) {}

  public static enum TargetLiftStates {
    STOW,
    L1,
    L2,
    L3,
    L4
  }

  private static TargetLiftStates nextState;
  private static TargetLiftStates curState;

  static final DutyCycleOut HOMING_SPEED = new DutyCycleOut(-0.05);
  static final Distance HOMING_EPSILON = Millimeters.of(5);

  // Tolerance in cm of top and bottom minimum clearance
  static final Distance ELEVATOR_TOLERANCE = Centimeters.of(0.5);

  // Tolerance in degrees of arm
  static final Angle ARM_TOLERANCE = Degrees.of(0.5);

  static final Angle SCORING_L1_ANGLE = Degrees.of(0);
  static final Angle SCORING_L2_ANGLE = Degrees.of(0);
  static final Angle SCORING_L3_ANGLE = Degrees.of(0);
  static final Angle SCORING_L4_ANGLE = Degrees.of(0);

  static final Angle SAFE_REEF_ANGLE_BOTTOM = Degrees.of(0);
  static final Angle SAFE_REEF_ANGLE_TOP = Degrees.of(0);
  static final Angle SAFE_INTAKE_ANGLE_BOTTOM = Degrees.of(0);
  static final Angle SAFE_INTAKE_ANGLE_TOP = Degrees.of(0);

  static final Angle STOW_ANGLE = Degrees.of(0);
  static final Distance STOW_HEIGHT = Inches.of(0);

  static final Distance L1_HEIGHT = Inches.of(0);
  static final Distance L2_HEIGHT = Inches.of(0);
  static final Distance CLEAR_HEIGHT = Inches.of(0);
  static final Distance L3_HEIGHT = Inches.of(0);
  static final Distance L4_HEIGHT = Inches.of(0);

  public enum LiftStates implements SystemState {
    IDLE {
      @Override
      public LiftStates nextState() {
        if (nextState == TargetLiftStates.STOW) {
          return STOW;
        }
        return this;
      }
    },
    HOME {
      @Override
      public void initialize() {
        s_liftinstance.startHomingElevator();
      }

      @Override
      public void end(boolean interrupted) {
        s_liftinstance.resetElevatorEncoder();
        s_liftinstance.stopElevator();
      }

      @Override
      public LiftStates nextState() {
        if (s_liftinstance.elevatorAtHome())
          return IDLE;
        return this;
      }
    },
    STOW {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(STOW_HEIGHT);
        s_liftinstance.setArmAngle(STOW_ANGLE);
      }

      @Override
      public SystemState nextState() {
        curState = TargetLiftStates.STOW;
        if (nextState == TargetLiftStates.L1) {
          return STOW_L1_S0;
        }
        if (nextState == TargetLiftStates.L2) {
          return STOW_L2_S0;
        }
        if (nextState == TargetLiftStates.L3) {
          return STOW_L3_S0;
        }
        if (nextState == TargetLiftStates.L4) {
          return STOW_L4_S0;
        }
        return this;
      }
    },
    STOW_L1_S0 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    STOW_L1_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L1 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L1_HEIGHT);
        s_liftinstance.setArmAngle(SCORING_L1_ANGLE);
      }

      @Override
      public SystemState nextState() {
        curState = TargetLiftStates.L1;
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.L2) {
          return L1_L2_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L1_L3_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L1_L4_S1;
        }
        return this;
      }
    },
    L1_STOW_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return L1_STOW_S2;
        }
        return this;
      }
    },
    L1_STOW_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(STOW_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(STOW_HEIGHT)) {
          return STOW;
        }
        return this;
      }
    },
    L1_L2_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L1_L2_S2;
        }
        return this;
      }
    },
    L1_L2_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L1_L3_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L1_L4_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L2_S0 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L2_S1;
        }
        return this;
      }
    },
    STOW_L2_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L2_HEIGHT);
        s_liftinstance.setArmAngle(SCORING_L2_ANGLE);
      }

      @Override
      public SystemState nextState() {
        curState = TargetLiftStates.L2;
        if (nextState == TargetLiftStates.STOW) {
          return L1_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L2_L1_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L2_L3_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L2_L4_S1;
        }
        return this;
      }
    },
    L2_L1_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    L2_L3_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L2_L4_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L3_S0 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    STOW_L3_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(CLEAR_HEIGHT)) {
          return STOW_L3_S2;
        }
        return this;
      }
    },
    STOW_L3_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().gte(SAFE_INTAKE_ANGLE_TOP)) {
          return STOW_L3_S3;
        }
        return this;
      }
    },
    STOW_L3_S3 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L3_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L3_HEIGHT)) {
          return L3;
        }
        return this;
      }
    },
    L3 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L3_HEIGHT);
        s_liftinstance.setArmAngle(SCORING_L3_ANGLE);
      }

      @Override
      public SystemState nextState() {
        curState = TargetLiftStates.L3;
        if (nextState == TargetLiftStates.STOW) {
          return L3_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L3_L1_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L3_L2_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L3_L4_S1;
        }
        return this;
      }
    },
    L3_STOW_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_STOW_S2;
        }
        return this;
      }
    },
    L3_STOW_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_STOW_S3;
        }
        return this;
      }
    },
    L3_STOW_S3 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L3_STOW_S4;
        }
        return this;
      }
    },
    L3_STOW_S4 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(STOW_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(STOW_HEIGHT)) {
          return STOW;
        }
        return this;
      }
    },
    L3_L1_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_TOP.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L1_S2;
        }
        return this;
      }
    },
    L3_L1_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_L1_S3;
        }
        return this;
      }
    },
    L3_L1_S3 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L3_L1_S4;
        }
        return this;
      }
    },
    L3_L1_S4 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L3_L2_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_TOP.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L2_S2;
        }
        return this;
      }
    },
    L3_L2_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(CLEAR_HEIGHT.plus(ELEVATOR_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L3_L2_S3;
        }
        return this;
      }
    },
    L3_L2_S3 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L3_L2_S4;
        }
        return this;
      }
    },
    L3_L2_S4 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L3_L4_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L3_L4_S2;
        }
        return this;
      }
    },
    L3_L4_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L4_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L4_HEIGHT)) {
          return L4;
        }
        return this;
      }
    },
    STOW_L4_S0 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L4_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L4_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getElevatorHeight().gte(CLEAR_HEIGHT)) {
          return L4;
        }
        return this;
      }
    },
    L4 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L4_HEIGHT);
        s_liftinstance.setArmAngle(SCORING_L4_ANGLE);
      }

      @Override
      public SystemState nextState() {
        curState = TargetLiftStates.L4;
        if (nextState == TargetLiftStates.STOW) {
          return L4_STOW_S1;
        }
        if (nextState == TargetLiftStates.L1) {
          return L4_L1_S1;
        }
        if (nextState == TargetLiftStates.L3) {
          return L4_L2_S1;
        }
        if (nextState == TargetLiftStates.L4) {
          return L4_L3_S1;
        }
        return this;
      }
    },
    L4_STOW_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_STOW_S2;
        }
        return this;
      }
    },
    L4_STOW_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_INTAKE_ANGLE_BOTTOM.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L1_STOW_S2;
        }
        return this;
      }
    },
    L4_L1_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_L1_S2;
        }
        return this;
      }
    },
    L4_L1_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L4_L2_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_BOTTOM.plus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_INTAKE_ANGLE_BOTTOM)) {
          return L4_L2_S2;
        }
        return this;
      }
    },
    L4_L2_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L2_HEIGHT)) {
          return L2;
        }
        return this;
      }
    },
    L4_L3_S1 {
      @Override
      public void initialize() {
        s_liftinstance.setArmAngle(SAFE_REEF_ANGLE_TOP.minus(ARM_TOLERANCE));
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.getArmAngle().lte(SAFE_REEF_ANGLE_TOP)) {
          return L4_L3_S2;
        }
        return this;
      }
    },
    L4_L3_S2 {
      @Override
      public void initialize() {
        s_liftinstance.setElevatorHeight(L3_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (s_liftinstance.elevatorAt(L3_HEIGHT)) {
          return L3;
        }
        return this;
      }
    };
  }

  private static LiftSubsystem s_liftinstance;

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_pivotMotor;
  private final MotionMagicVoltage m_pivotPositionSetter;
  private final MotionMagicVoltage m_elevatorPositionSetter;
  private final LimitSwitch m_elevatorHomingBeamBreak;
  private final Consumer<SysIdRoutineLog.State> m_elevatorSysIDLogConsumer;
  private final Consumer<SysIdRoutineLog.State> m_pivotSysIDLogConsumer;
  private static final String ELEVATOR_MOTOR_SYSID_STATE_LOG_ENTRY = "/ElevatorMotorSysIDTestState";
  private static final String PIVOT_MOTOR_SYSID_STATE_LOG_ENTRY = "/PivotMotorSysIDTestState";


  /** Creates a new LiftSubsystem */
  private LiftSubsystem(Hardware liftHardware) {
    super(LiftStates.IDLE);
    m_elevatorMotor = liftHardware.elevatorMotor;
    m_pivotMotor = liftHardware.pivotMotor;
    m_pivotPositionSetter = new MotionMagicVoltage(Radians.zero());
    m_elevatorPositionSetter = new MotionMagicVoltage(Radians.zero());
    m_elevatorHomingBeamBreak = liftHardware.elevatorHomingBeamBreak;

    // TODO: Figure out motor configuration values (gear ratios, sensors, etc)

    // Create configurations for elevator motor
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    elevatorConfig.Feedback.SensorToMechanismRatio = 5.0;
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

    // Create configurations for pivot motor
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
    pivotConfig.Feedback.RotorToSensorRatio = 52.36363636363636;
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

    m_elevatorSysIDLogConsumer = state -> SignalLogger.writeString(getName() + ELEVATOR_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());
    m_pivotSysIDLogConsumer = state -> SignalLogger.writeString(getName() + PIVOT_MOTOR_SYSID_STATE_LOG_ENTRY, state.toString());

    // Apply configs for TalonFX motors
    m_elevatorMotor.applyConfigs(elevatorConfig);
    m_pivotMotor.applyConfigs(pivotConfig);
  }

  /**
   * Get an instance of LiftSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   * @param LiftHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static LiftSubsystem getInstance(Hardware liftHardware) {
    if (s_liftinstance == null) {
      s_liftinstance = new LiftSubsystem(liftHardware);
      return s_liftinstance;
    } else return null;
  }

  /**
   * Initialize hardware devices for lift subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware liftHardware = new Hardware(
      new TalonFX(Constants.LiftHardware.ELEVATOR_MOTOR_ID, Constants.Frequencies.TALON_UPDATE_RATE),
      new TalonFX(Constants.LiftHardware.PIVOT_MOTOR_ID, Constants.Frequencies.TALON_UPDATE_RATE),
      new LimitSwitch(Constants.LiftHardware.ELEVATOR_HOMING_BEAM_BREAK_PORT, SwitchPolarity.NORMALLY_OPEN, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE)
    );

    return liftHardware;
  }

  /**
   * Set arm pivot to a certain angle
   * @param angle The angle you want to move the pivot
   */
  public void setArmAngle(Angle angle) {
    m_pivotMotor.setControl(m_pivotPositionSetter.withPosition(angle));
  }

  /**
   * Set elevator to a certain height
   * @param height The height you want to move the elevator to
   */
  public void setElevatorHeight(Distance height) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);
    m_elevatorMotor.setControl(m_elevatorPositionSetter.withPosition(elevatorMoveAngle));
  }

  /**
   * Get current arm angle
   */
  private Angle getArmAngle() {
    return m_pivotMotor.getInputs().rotorPosition;
  }

  /**
   * Get current elevator height
   */
  private Distance getElevatorHeight() {
    Angle elevatorAngle = m_elevatorMotor.getInputs().rotorPosition;
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Distance height = Meters.of(circumference * elevatorAngle.in(Rotations));
    return height;
  }

  /**
   * Slowly run the elevator motor
   */
  private void startHomingElevator() {
    m_elevatorMotor.setControl(HOMING_SPEED);
  }

  /**
   * Zero the elevator encoder
   */
  private void resetElevatorEncoder() {
    m_elevatorMotor.setPosition(Degrees.of(0.0));
  }

  /**
   * Check if elevator is at home
   * @return True if elevator is home
   */
  public boolean elevatorAtHome() {
    return m_elevatorHomingBeamBreak.getInputs().value;
  }

  /**
   * Return whether the elevator is at a target height or not
   * @return Boolean of if elevator is at target height
   */
  public boolean elevatorAt(Distance targetHeight) {
    Distance currentHeight = getElevatorHeight();
    return (currentHeight.isNear(currentHeight, ELEVATOR_TOLERANCE));
  }

  /**
   * Return whether the arm is at a target angle or not
   * @return Boolean of if arm is at target angle
   */
  public boolean armAt(Angle targetAngle) {
    Angle currentAngle = getArmAngle();
    return (currentAngle.isNear(targetAngle, ARM_TOLERANCE));
  }

  /**
   * Gets the SysID routine for the elevator
   */
  public SysIdRoutine getElevatorSysIDRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,
        m_elevatorSysIDLogConsumer
      ),
      new SysIdRoutine.Mechanism(
      voltage -> m_elevatorMotor.setControl(new VoltageOut(voltage)),
      null, s_liftinstance)
    );
  }


  /**
   *
   * @return
   */
  public SysIdRoutine getPivotSysIDRoutine() {
    return new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,
        m_pivotSysIDLogConsumer
      ),
      new SysIdRoutine.Mechanism(
      voltage -> m_pivotMotor.setControl(new VoltageOut(voltage)),
      null, s_liftinstance)
    );
  }


  /**
   * Stop the elevator motor
   */
  private void stopElevator() {
    m_elevatorMotor.stopMotor();
  }

  /**
   * Close the motors of the lift subsystem, make the instance null
   */
  public void close() {
    m_elevatorMotor.close();
    m_pivotMotor.close();
    s_liftinstance = null;
  }

  /**
   * Set state of lift state machine
   * @param state The target TargetLiftStates state to go to
   */
  public void setState(TargetLiftStates state) {
    nextState = state;
  }

  /**
   * See if the current nextState is at a given state
   * @param state The LiftStates state to check against
   */
  public boolean isAtState(TargetLiftStates state) {
    return curState == state;
  }
}
