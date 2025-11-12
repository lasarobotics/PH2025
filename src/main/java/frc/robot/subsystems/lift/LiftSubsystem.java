package frc.robot.subsystems.lift;

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
import org.lasarobotics.hardware.ctre.CANcoder;
import org.lasarobotics.hardware.generic.LimitSwitch;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LoopTimer;

import frc.robot.subsystems.lift.LiftInstructions.*;

public class LiftSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware (
    TalonFX elevatorMotor,
    TalonFX pivotMotor,
    LimitSwitch elevatorHomingBeamBreak,
    CANcoder armCANCoder
  ) {}

  public static enum TargetLiftStates {
    NOTHING,
    TURBO,
    STOW,
    L1,
    L2,
    L3,
    L4,
    PANIC,
    A1,
    A2,
    A_SCORE
  }

  private static TargetLiftStates nextState;
  private static TargetLiftStates curState;
  private static IDF[] currentInstructionSet;
  private static boolean isLiftReady;
  private static boolean isDisabled;

  static final DutyCycleOut HOMING_SPEED = new DutyCycleOut(0.05);
  static final Distance HOMING_EPSILON = Millimeters.of(5);

  // Tolerance in cm of top and bottom minimum clearance
  static final Distance ELEVATOR_TOLERANCE = Inches.of(2.0);

  // Tolerance in degrees of arm
  static final Angle ARM_TOLERANCE = Degrees.of(2.0);

  static final Angle SAFE_REEF_ANGLE_BOTTOM = Rotations.of(-0.256836);
  static final Angle SAFE_REEF_ANGLE_TOP = Rotations.of(0.287598-0.0278);

  static final Angle STOW_ANGLE = Rotations.of(-0.215333);
  static final Distance STOW_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(0.05));

  static final Distance BEAM_BREAK_HEIGHT = LiftSubsystem.convertToDistance(Rotations.of(0));

  public enum LiftStates implements SystemState {
    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }

    },
    DISABLED {
      @Override
      public void initialize() {
        s_liftinstance.stopElevator();
        s_liftinstance.stopArm();
        isLiftReady = true; // Done so that the entire robot can keep working, even though the lift is disabled
      }

      @Override
      public LiftStates nextState() {
        // Intentionally designed to have no escape, robot restart needed
        return this;
      }
    },
    /*
    HOME {
      private boolean isDoneHoming = false;

      @Override
      public void initialize() {
        if (s_liftinstance.getArmAngle().lte(STOW_ANGLE.plus(ARM_TOLERANCE)) && s_liftinstance.getArmAngle().gte(SAFE_REEF_ANGLE_BOTTOM.minus(ARM_TOLERANCE)) && s_liftinstance.elevatorAtHome()) {
          s_liftinstance.startHomingElevator();
        } else {
          isDisabled = true;
        }
      }

      @Override
      public void execute() {
        if (!s_liftinstance.elevatorAtHome()) { // There is a ! on this line.
          s_liftinstance.setElevatorEncoder(BEAM_BREAK_HEIGHT);
          isDoneHoming = true;
        }
      }

      @Override
      public void end(boolean interrupted) {
        s_liftinstance.stopElevator();
      }

      @Override
      public LiftStates nextState() {
        if (isDisabled) {
          return DISABLED;
        }
        if (isDoneHoming) {
          currentInstructionSet = LiftInstructions.INIT_STOW_INSTRUCTIONS;
          curState = TargetLiftStates.STOW;
          nextState = TargetLiftStates.STOW;
          return TRANSITION;
        }
        return this;
      }
    },
    */
    TRANSITION {
      IDF[] instructionSet;
      int currentStep;
      boolean stepInitialized;

      @Override
      public void initialize() {
        isLiftReady = false;
        currentStep = 0;
        stepInitialized = false;
        instructionSet = currentInstructionSet;
      }

      @Override
      public void execute() {
        if (currentStep >= instructionSet.length) {
          isLiftReady = true;
          return;
        }
        IDF currentInstruction = instructionSet[currentStep];
        if (!stepInitialized) {
          s_liftinstance.executeInstruction(currentInstruction);
          stepInitialized = true;
        } else if (s_liftinstance.checkInstruction(currentInstruction)) {
          currentStep++;
          stepInitialized = false;
        }
      }

      @Override
      public SystemState nextState() {
        if (currentStep >= instructionSet.length) {
          curState = nextState;
          return AT_STATE;
        }
        return this;
      }
    },
    AT_STATE {
      @Override
      public SystemState nextState() {
        // mapStatesToTransition returns null if identical
        IDF[] idf = LiftInstructions.mapStatesToTransition(curState, nextState);
        if (idf != null) {
          currentInstructionSet = idf;
          return TRANSITION;
        } else {
          return this;
        }
      }
    }
  }

  private static LiftSubsystem s_liftinstance;

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_pivotMotor;
  private final CANcoder m_armCANcoder;
  private final MotionMagicVoltage m_pivotPositionSetter;
  private final MotionMagicVoltage m_elevatorPositionSetter;
  private final LimitSwitch m_elevatorHomingBeamBreak;
  private final Consumer<SysIdRoutineLog.State> m_elevatorSysIDLogConsumer;
  private final Consumer<SysIdRoutineLog.State> m_pivotSysIDLogConsumer;
  private static final String ELEVATOR_MOTOR_SYSID_STATE_LOG_ENTRY = "/ElevatorMotorSysIDTestState";
  private static final String PIVOT_MOTOR_SYSID_STATE_LOG_ENTRY = "/PivotMotorSysIDTestState";

  /** Creates a new LiftSubsystem */
  private LiftSubsystem(Hardware liftHardware) {
    super(LiftStates.TRANSITION);
    // curState doesn't really matter here, but it doesn't hurt to have it
    curState = TargetLiftStates.STOW;
    nextState = TargetLiftStates.STOW;
    currentInstructionSet = LiftInstructions.INIT_STOW_INSTRUCTIONS;
    m_elevatorMotor = liftHardware.elevatorMotor;
    m_pivotMotor = liftHardware.pivotMotor;
    m_armCANcoder = liftHardware.armCANCoder;

    m_pivotPositionSetter = new MotionMagicVoltage(Radians.zero());
    m_elevatorPositionSetter = new MotionMagicVoltage(Radians.zero());
    m_elevatorHomingBeamBreak = liftHardware.elevatorHomingBeamBreak;

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
    pivotConfig.Feedback.FeedbackRemoteSensorID = m_armCANcoder.getID().deviceID;
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

    m_elevatorSysIDLogConsumer = state -> SignalLogger.writeString(getName() + ELEVATOR_MOTOR_SYSID_STATE_LOG_ENTRY,
        state.toString());
    m_pivotSysIDLogConsumer = state -> SignalLogger.writeString(getName() + PIVOT_MOTOR_SYSID_STATE_LOG_ENTRY,
        state.toString());

    // Apply configs for TalonFX motors
    m_elevatorMotor.getConfigurator().apply(elevatorConfig);
    m_pivotMotor.getConfigurator().apply(pivotConfig);
    m_armCANcoder.applyConfigs(armCANCoderConfig);
  }

  /**
   * Get an instance of LiftSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   *
   * @param LiftHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static LiftSubsystem getInstance(Hardware liftHardware) {
    if (s_liftinstance == null) {
      s_liftinstance = new LiftSubsystem(liftHardware);
      return s_liftinstance;
    } else
      return null;
  }

  /**
   * Initialize hardware devices for lift subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware liftHardware = new Hardware(
      new TalonFX(Constants.LiftHardware.ELEVATOR_MOTOR_ID.deviceID, Constants.LiftHardware.ELEVATOR_MOTOR_ID.bus.name),
      new TalonFX(Constants.LiftHardware.PIVOT_MOTOR_ID.deviceID, Constants.LiftHardware.PIVOT_MOTOR_ID.bus.name),
      new LimitSwitch(Constants.LiftHardware.ELEVATOR_HOMING_BEAM_BREAK_PORT, Constants.Frequencies.BEAM_BREAK_UPDATE_RATE),
      new CANcoder(Constants.LiftHardware.ARM_CANCODER_ID, Constants.Frequencies.TALON_UPDATE_RATE)
    );

    return liftHardware;
  }

  /**
   * Set the arm and elevator to the positions specified in the instruction
   * 
   * @param instruction The instruction containing the target positions
   */
  private void executeInstruction(IDF instruction) {
    if (instruction.wantedArmAngle() != null) {
      setArmAngle(instruction.wantedArmAngle());
    }
    if (instruction.wantedElevatorHeight() != null) {
      setElevatorHeight(instruction.wantedElevatorHeight());
    }
  }

  /**
   * Check if the current arm angle and elevator height meet the conditions in the instruction
   * 
   * @param instruction The instruction containing the target conditions
   * @return If the conditions in the instruction are met
   */
  private boolean checkInstruction(IDF instruction) {
    return (instruction.wantedArmAngle() == null || instruction.armComparison().compare(getArmAngle()))
  && (instruction.wantedElevatorHeight() == null || instruction.elevatorComparison().compare(getElevatorHeight()));
  }

  /**
   * Set arm pivot to a certain angle
   *
   * @param angle The angle you want to move the pivot
   */
  private void setArmAngle(Angle angle) {
    Logger.recordOutput(getName() + "/targetArmAngle", angle.in(Rotations));
    m_pivotMotor.setControl(m_pivotPositionSetter.withPosition(angle));
  }

  /**
   * Set elevator to a certain height
   *
   * @param height The height you want to move the elevator to
   */
  private void setElevatorHeight(Distance height) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);

    Logger.recordOutput(getName() + "/targetElevatorHeight", height);
    Logger.recordOutput(getName() + "/targetElevatorAngle", elevatorMoveAngle.in(Rotations));
    m_elevatorMotor.setControl(m_elevatorPositionSetter.withPosition(elevatorMoveAngle));
  }

  /**
   * Get current arm angle
   */
  public Angle getArmAngle() {
    return m_armCANcoder.getInputs().absolutePosition;
  }

  /**
   * Get current arm velocity
   */
  public AngularVelocity getArmVelocity() {
    return m_pivotMotor.getVelocity().getValue();
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
  public Distance getElevatorHeight() {
    return LiftSubsystem.convertToDistance(m_elevatorMotor.getPosition().getValue());
  }

  /**
   * Slowly run the elevator motor to home
   */
  private void startHomingElevator() {
    m_elevatorMotor.setControl(HOMING_SPEED);
  }

  /**
   * Set the elevator encoder to a given height
   */
  private void setElevatorEncoder(Distance height) {
    Distance SPROCKET_RADIUS = Constants.LiftHardware.SPROCKET_PITCH_RADIUS;
    double circumference = 2 * Math.PI * SPROCKET_RADIUS.in(Meters);
    Angle elevatorMoveAngle = Rotations.of(height.in(Meters) / circumference);
    m_elevatorMotor.setPosition(elevatorMoveAngle);
  }

  /**
   * Check if elevator is at home
   *
   * @return True if elevator is home
   */
  public boolean elevatorAtHome() {
    return elevatorHomingBeamBreak();
  }

  /**
   * Return whether the lift is ready to move or not
   * @return Boolean of if lift is at one of the 5 stages
   */
  public boolean isLiftReady() {
    return isLiftReady;
  }

  /**
   * Return if the elevator's homing beam break is broken
   * @return Boolean for if the elevator's homing beam break is broken
   */
  private boolean elevatorHomingBeamBreak() {
    return !m_elevatorHomingBeamBreak.getInputs().value;
  }

  /**
   * Gets the SysID routine for the elevator
   * @return SysID routine for the elvator
   */
  public SysIdRoutine getElevatorSysIDRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            m_elevatorSysIDLogConsumer),
        new SysIdRoutine.Mechanism(
            voltage -> m_elevatorMotor.setControl(new VoltageOut(voltage)),
            null, s_liftinstance));
  }

  /**
   * Gets the SysID routine for the pivot/arm
   * @return Returns the SysID routine for the pivot/arm
   */
  public SysIdRoutine getPivotSysIDRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            m_pivotSysIDLogConsumer),
        new SysIdRoutine.Mechanism(
            voltage -> m_pivotMotor.setControl(new VoltageOut(voltage)),
            null, s_liftinstance));
  }

  /**
   * Stop the elevator motor
   */
  private void stopElevator() {
    m_elevatorMotor.stopMotor();
  }

  /**
   * Stop the arm motor
   */
  private void stopArm() {
    m_pivotMotor.stopMotor();
  }

  /**
   * Set state of lift state machine for API purposes
   * @param state The target TargetLiftStates state to go to
   */
  public void setState(TargetLiftStates state) {
    nextState = state;
  }

  /**
   * See if the current nextState is at a given state
   *
   * @param state The LiftStates state to check against
   */
  public boolean isAtState(TargetLiftStates state) {
    return curState == state;
  }


  @Override
  public void periodic() {
    LoopTimer.addTimestamp(getName() + " Start");
    super.periodic();

    Logger.recordOutput(getName() + "/state", getState().toString());
    Logger.recordOutput(getName() + "/homingBeamBreak", elevatorHomingBeamBreak());

    Logger.recordOutput(getName() + "/currentArmAngle", getArmAngle().in(Rotations));
    Logger.recordOutput(getName() + "/currentElevatorHeight", getElevatorHeight());
    Logger.recordOutput(getName() + "/currentElevatorAngle", m_elevatorMotor.getPosition().getValue().in(Rotations));
    Logger.recordOutput(getName() + "/isLiftReady", this.isLiftReady());
    LoopTimer.addTimestamp(getName() + " End");
  }

  /**
   * Close the motors of the lift subsystem, make the instance null
   */
  @Override
  public void close() {
    m_elevatorMotor.close();
    m_pivotMotor.close();
    s_liftinstance = null;
  }
}
