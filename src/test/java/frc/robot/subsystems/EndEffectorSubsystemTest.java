package frc.robot.subsystems;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import static org.mockito.ArgumentMatchers.doubleThat;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.MAXSwerveModule.ModuleLocation;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.kauailabs.NavX2InputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.utils.GlobalConstants;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;


import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

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

        when(m_endEffectorMotor.getMotorType()).thenReturn(MotorKind.NEO_VORTEX);

        when(m_endEffectorMotor.getID()).thenReturn(Constants.EndEffectorHardware.OUTTAKE_MOTOR_ID);

        m_endEffectorHardware = new EndEffectorSubsystem.Hardware(m_endEffectorMotor);

        m_endEffectorSystem = new EndEffectorSubsystem(m_endEffectorHardware);
    }

    public void close() {
        m_endEffectorSystem.close();
        m_endEffectorSystem = null;
    }

    private SparkInputsAutoLogged getRotateSparkInput(Rotation2d rotation, ModuleLocation moduleLocation) {
        var sparkInputs = new SparkInputsAutoLogged();
        sparkInputs.absoluteEncoderPosition = rotation.minus(moduleLocation.offset).getRadians();
    
        return sparkInputs;
    }

    @Test
    @Order(1)
    @DisplayName("Test if End Effector can outtake")
    public void outtake() {
        SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();

        m_endEffectorSystem.setState(EndEffectorSubsystem.endEffectorStates.SCORE);

        verify(m_endEffectorMotor, times(1)).set(1.0);
    }
}
