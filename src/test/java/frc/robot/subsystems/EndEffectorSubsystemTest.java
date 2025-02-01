package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class EndEffectorSubsystemTest {
  private EndEffectorSubsystem m_endEffectorSystem;
  private EndEffectorSubsystem.Hardware m_endEffectorHardware;
  
  private Spark m_endEffectorMotor;
  
  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
    
    m_endEffectorMotor = mock(Spark.class);
    
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_endEffectorMotor.getInputs()).thenReturn(sparkInputs);
    
    when(m_endEffectorMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);
    
    when(m_endEffectorMotor.getID()).thenReturn(Constants.EndEffectorHardware.OUTTAKE_MOTOR_ID);
    
    m_endEffectorHardware = new EndEffectorSubsystem.Hardware(m_endEffectorMotor);
    
    m_endEffectorSystem = EndEffectorSubsystem.getInstance(m_endEffectorHardware);
  }
  
  public void close() {
    m_endEffectorSystem.close();
    m_endEffectorSystem = null;
  }
  
  @Test
  @Order(1)
  @DisplayName("Test if End Effector can outtake")
  public void outtake() {
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    
    m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.SCORE);
    
    when(m_endEffectorMotor.getInputs()).thenReturn(sparkInputs);
    
    verify(m_endEffectorMotor, times(1)).set(1.0);

    m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.IDLE);

  }
  
  @Test
  @Order(2)
  @DisplayName("Test if End Effector can regurgitate")
  public void regurgitate() {
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    
    m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.REGURGITATE);
    
    when(m_endEffectorMotor.getInputs()).thenReturn(sparkInputs);
    
    verify(m_endEffectorMotor, times(1)).set(-1.0);

    m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.IDLE);

  }

  @Test
  @Order(3)
  @DisplayName("Test if End Effector can Intake")
  public void intake() {
    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();

    sparkInputs.forwardLimitSwitch = false;
    sparkInputs.reverseLimitSwitch = false;

    m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.INTAKE);

    verify(m_endEffectorMotor, times(1)).set(Constants.EndEffector.INTAKE_MOTOR_SPEED);

    sparkInputs.forwardLimitSwitch = true;
    sparkInputs.reverseLimitSwitch = false;

    verify(m_endEffectorMotor, times(1)).set(Constants.EndEffector.CENTER_CORAL_MOTOR_SPEED);

    sparkInputs.forwardLimitSwitch = true;
    sparkInputs.reverseLimitSwitch = true;

    verify(m_endEffectorMotor, times(1)).close();



  }
}
