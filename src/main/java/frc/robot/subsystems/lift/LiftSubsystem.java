package frc.robot.subsystems.lift;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SystemState;
import frc.robot.Constants;


public class LiftSubsystem extends StateMachine implements AutoCloseable {
	public static record Hardware (TalonFX elevatorMotor, TalonFX pivotMotor, Spark outtakeMotor) {}

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

  /**Creates a new LiftSubsystem. */
	private LiftSubsystem(Hardware liftHardware) {
		super(LiftStates.IDLE);
		this.m_elevatorMotor = liftHardware.elevatorMotor;
		this.m_pivotMotor = liftHardware.pivotMotor;
		this.m_outtakeMotor = liftHardware.outtakeMotor;

		//TODO: figure out motor configurations

		//Create configurations for each motor
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

		//Apply default configs FOR SPECIFICALLY SparkFlex outtake
		m_outtakeMotor.restoreFactoryDefaults();
		

		//Set idle mode
		elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_outtakeMotor.setIdleMode(IdleMode.kBrake);


		//Apply configs 
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
   * Initialize hardware devices for lif tsubsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
	public static Hardware initializeHardware() {
		Hardware liftHardware = new Hardware(
			new TalonFX(Constants.Lift.ELEVATOR_MOTOR_ID),
			new TalonFX(Constants.Lift.PIVOT_MOTOR_ID),
			new Spark(Constants.Lift.OUTTAKE_MOTOR_ID, MotorKind.NEO_VORTEX)
		);
		return liftHardware;
	}

	public void close() {
		m_elevatorMotor.close();
		m_pivotMotor.close();
		m_outtakeMotor.close();
		s_liftinstance = null;
	}
}
