package frc.robot.subsystems.music;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;

/**
 * Lets you play music on the lift motor, a very necessary feature that won't get annoying
 */
public class MotorPitchSubsystem implements Subsystem, AutoCloseable {
    private static MotorPitchSubsystem s_motorPitchInstance = null;
    public static MotorPitchSubsystem getInstance(Hardware hardware) {
    if (s_motorPitchInstance == null) {
        s_motorPitchInstance = new MotorPitchSubsystem(hardware);
        return s_motorPitchInstance;
    } else return null;
  }

    public record Hardware(ParentDevice motor) {}

    public ParentDevice m_motor;
    Orchestra m_orchestra = new Orchestra(List.of(m_motor));

    private MotorPitchSubsystem(Hardware hardware) {
        this.m_motor = hardware.motor;
    }

    public Command playFile(String path) {
        return Commands.startEnd(
            () -> {
                m_orchestra.clearInstruments();
                m_orchestra.addInstrument(m_motor);
                m_orchestra.loadMusic(path);
            }, () -> {
                m_orchestra.stop();
            }
        );
    }

    public static Hardware initializeHardware() {
        return new Hardware(
            new CANcoder(Constants.LiftHardware.ELEVATOR_MOTOR_ID.deviceID)
        );
    }

    @Override
    public void close() throws Exception {
        m_orchestra.stop();
    }
}
