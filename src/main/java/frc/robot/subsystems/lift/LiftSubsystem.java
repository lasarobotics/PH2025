package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

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

  public static enum USER_INPUT {
    STOW,
    L1,
    L2,
    L3,
    L4
  }

  public static final Angle SCORING_L1_ANGLE = Degrees.of(0);
  public static final Angle SCORING_L2_ANGLE = Degrees.of(0);
  public static final Angle SCORING_L3_ANGLE = Degrees.of(0);
  public static final Angle SCORING_L4_ANGLE = Degrees.of(0);

  public static final Angle SAFE_REEF_ANGLE_BOTTOM = Degrees.of(0);
  public static final Angle SAFE_REEF_ANGLE_TOP = Degrees.of(0);
  public static final Angle SAFE_INTAKE_ANGLE = Degrees.of(0);

  public static final Angle STOW_ANGLE = Degrees.of(0);
  public static final Distance STOW_HEIGHT = Inches.of(0);

  public static final Distance L1_HEIGHT = Inches.of(0);
  public static final Distance L2_HEIGHT = Inches.of(0);
  public static final Distance CLEAR_HEIGHT = Inches.of(0);
  public static final Distance L3_HEIGHT = Inches.of(0);
  public static final Distance L4_HEIGHT = Inches.of(0);

  /* TODO: Actually get user input */
  private static USER_INPUT getUserInput() { return USER_INPUT.STOW; }

  public enum LiftStates implements SystemState {
    IDLE {
      @Override
      public void initialize() {

      }

      @Override
      public LiftStates nextState() {
        return this;
      }
    },
    STOW {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(STOW_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getUserInput() == USER_INPUT.L1) {
          return STOW_L1_S0;
        }
        if (getUserInput() == USER_INPUT.L2) {
          return STOW_L2_S0;
        }
        if (getUserInput() == USER_INPUT.L3) {
          return STOW_L3_S0;
        }
        if (getUserInput() == USER_INPUT.L4) {
          return STOW_L4_S0;
        }
        return this;
      }
    },
    STOW_L1_S0 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_INTAKE_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle().gte(SAFE_INTAKE_ANGLE)) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    STOW_L2_S0 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_INTAKE_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle().gte(SAFE_INTAKE_ANGLE)) {
          return STOW_L2_S1;
        }
        return this;
      }
    },
    STOW_L3_S0 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_INTAKE_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle().gte(SAFE_INTAKE_ANGLE)) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    STOW_L4_S0 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_INTAKE_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle().gte(SAFE_INTAKE_ANGLE)) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    STOW_L1_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(L1_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight().gte(L1_HEIGHT)) {
          return L1;
        }
        return this;
      }
    },
    L1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SCORING_L1_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getUserInput() == USER_INPUT.STOW) {
          return L1_STOW_S1;
        }
        if (getUserInput() == USER_INPUT.L2) {
          return L2;
        }
        if (getUserInput() == USER_INPUT.L3) {
          return L1_L3_S1;
        }
        if (getUserInput() == USER_INPUT.L4) {
          return L1_L4_S1;
        }
        return this;
      }
    },
    STOW_L2_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(L2_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == L2_HEIGHT) {
          return L2;
        }
        return this;
      }
    },
    L2 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SCORING_L2_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getUserInput() == USER_INPUT.STOW) {
          return L2_STOW_S1;
        }
        if (getUserInput() == USER_INPUT.L1) {
          return L2_L1_S1;
        }
        if (getUserInput() == USER_INPUT.L3) {
          return L2_L3_S1;
        }
        if (getUserInput() == USER_INPUT.L4) {
          return L2_L4_S1;
        }
        return this;
      }
    },
    STOW_L3_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == CLEAR_HEIGHT) {
          return STOW_L3_S2;
        }
        return this;
      }
    },
    STOW_L3_S2 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SCORING_L3_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SCORING_L3_ANGLE) {
          return L3;
        }
        return this;
      }
    },
    L3 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(L3_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getUserInput() == USER_INPUT.STOW) {
          return L3_STOW_S1;
        }
        if (getUserInput() == USER_INPUT.L1) {
          return L3_L1_S1;
        }
        if (getUserInput() == USER_INPUT.L3) {
          // return L3_L2_S1; TODO: Implement
        }
        if (getUserInput() == USER_INPUT.L4) {
          // return L3_L4_S1; TODO: Implement
        }
        return this;
      }
    },
    STOW_L4_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(L4_HEIGHT);
      }

      @Override
      public void execute() {
        if (getInstance(initializeHardware()).getElevatorHeight() == CLEAR_HEIGHT && getInstance(initializeHardware()).getArmAngle() == STOW_ANGLE) {
          getInstance(initializeHardware()).setArmAngle(SCORING_L4_ANGLE);
        }
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == L4_HEIGHT && getInstance(initializeHardware()).getArmAngle() == SCORING_L4_ANGLE) {
          return L4;
        }
        return this;
      }
    },
    L4 {
      @Override
      public SystemState nextState() {
        if (getUserInput() == USER_INPUT.STOW) {
          return L4_STOW_S1;
        }
        if (getUserInput() == USER_INPUT.L1) {
          // return L4_L1_S1; TODO: Implement
        }
        if (getUserInput() == USER_INPUT.L3) {
          // return L4_L2_S1; TODO: Implement
        }
        if (getUserInput() == USER_INPUT.L4) {
          // return L4_L3_S1; TODO: Implement
        }
        return this;
      }
    },
    L1_STOW_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(STOW_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == STOW_ANGLE) {
          return STOW;
        }
        return this;
      }
    },
    L2_STOW_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(STOW_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == STOW_ANGLE) {
          return STOW;
        }
        return this;
      }
    },
    L3_STOW_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == CLEAR_HEIGHT) {
          return L3_STOW_S2;
        }
        return this;
      }
    },
    L3_STOW_S2 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(STOW_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == STOW_ANGLE) {
          return STOW;
        }
        return this;
      }
    },
    L4_STOW_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(STOW_ANGLE);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == STOW_ANGLE) {
          return STOW;
        }
        return this;
      }
    },
    L1_L3_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_BOTTOM) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L1_L4_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_BOTTOM) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    L2_L1_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_BOTTOM) {
          return STOW_L1_S1;
        }
        return this;
      }
    },
    L2_L3_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_BOTTOM) {
          return STOW_L3_S1;
        }
        return this;
      }
    },
    L2_L4_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_BOTTOM);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_BOTTOM) {
          return STOW_L4_S1;
        }
        return this;
      }
    },
    L3_L1_S1 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setArmAngle(SAFE_REEF_ANGLE_TOP);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getArmAngle() == SAFE_REEF_ANGLE_TOP) {
          return L3_L1_S2;
        }
        return this;
      }
    },
    L3_L1_S2 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == CLEAR_HEIGHT) {
          return L3_L1_S3;
        }
        return this;
      }
    },
    L3_L1_S3 {
      @Override
      public void initialize() {
        getInstance(initializeHardware()).setElevatorHeight(CLEAR_HEIGHT);
      }

      @Override
      public SystemState nextState() {
        if (getInstance(initializeHardware()).getElevatorHeight() == CLEAR_HEIGHT) {
          return L3_L1_S3;
        }
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

  /** Creates a new LiftSubsystem */
  private LiftSubsystem(Hardware liftHardware) {
    super(LiftStates.IDLE);
    this.m_elevatorMotor = liftHardware.elevatorMotor;
    this.m_pivotMotor = liftHardware.pivotMotor;
    this.m_outtakeMotor = liftHardware.outtakeMotor;
    this.m_insideEndEffectorBeamBreak = liftHardware.insideEndEffectorBeamBreak;
    this.m_outsideEndEffectorBeamBreak = liftHardware.outsideEndEffectorBeamBreak;
    this.m_elevatorHomingBeamBreak = liftHardware.elevatorHomingBeamBreak;

    //TODO: Figure out motor configuration values (gear ratios, sensors, etc)

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

    // Apply SparkFlex configs
    SparkMaxConfig outtakeConfig = new SparkMaxConfig();
    outtakeConfig.smartCurrentLimit(0); // To-do constants
    m_outtakeMotor.configure(outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Apply configs for TalonFX motors
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
   * Set arm pivot to a certain angle
   * @param angle The angle you want to move the pivot
   */
  public void setArmAngle(Angle angle) {
    m_pivotMotor.setControl(new MotionMagicVoltage(angle));
  }

  /**
   * Set elevator to a certain height
   * @param height The height you want to move the elevator to
   */
  public void setElevatorHeight(Distance height) {
    Distance SPROCKET_RADIUS = Constants.Lift.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);
    m_elevatorMotor.setControl(new MotionMagicVoltage(elevatorMoveAngle));
  }

  /**
   * Get current arm angle
   */
  public Angle getArmAngle() {
    return m_pivotMotor.getPosition().getValue();
  }

  /**
   * Get current elevator height
   */
  public Distance getElevatorHeight() {
    Angle elevatorAngle = m_elevatorMotor.getPosition().getValue();
    Distance SPROCKET_RADIUS = Constants.Lift.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Distance height = Meters.of(circumference * elevatorAngle.in(Rotations));
    return height;
  }

  /**
   * Run the scoring outtake to score coral on the reef
   * @param dutyCycleOutput The dutycycle output to set the motor at for outtake
   */
  public void runScoringMech(double dutyCycleOutput) {
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
