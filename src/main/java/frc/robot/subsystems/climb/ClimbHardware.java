package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public final class ClimbHardware {

  static final double CLIMB_SPEED = 1.0;
  static final double CLIMB_SPEED_SLOW = CLIMB_SPEED * 0.2;
  static final double MOUNT_ANGLE = 0.357;
  static final double CLIMB_ANGLE = 0.12;
  static final double STOW_ANGLE = 0.08;
  static final double SERVO_UNLOCK_ANGLE = 180.0;
  static final double SERVO_LOCK_ANGLE = 45.0;

  private static Spark s_climbEncoder;
  private static TalonFX s_climbMotor;
  private static TalonFXConfiguration s_climbMotorConfig;
  private static Servo s_latchServo;

  /** Creates a new ClimbSubsystem. */
  public static void initialize() {
    s_climbEncoder = new Spark(
        Constants.ClimbHardware.ENCODER_ID,
        MotorKind.NEO
    );
    s_climbMotor = new TalonFX(
        Constants.ClimbHardware.CLIMB_MOTOR_ID.deviceID,
        Constants.ClimbHardware.CLIMB_MOTOR_ID.bus.name
    );
    s_latchServo = new Servo(
        Constants.ClimbHardware.SERVO_ID
    );

    s_climbMotorConfig = new TalonFXConfiguration();
    s_climbMotorConfig.MotorOutput.NeutralMode = 
      NeutralModeValue.Brake;

    s_climbMotor.getConfigurator().apply(
      s_climbMotorConfig
    );
  }

  public static void stopAll() {
    s_climbMotor.stopMotor();
    s_latchServo.setDisabled();
  }

  public static void stopClimber() {
    s_climbMotor.stopMotor();
  }

  public static void climb() {
    s_climbMotor.set(CLIMB_SPEED);
  }

  public static void mount() {
    s_climbMotor.set(-CLIMB_SPEED);
  }

  public static void stow() {
    s_climbMotor.set(CLIMB_SPEED_SLOW);
  }

  public static double getClimberPosition() {
    return s_climbEncoder.getInputs().absoluteEncoderPosition;
  }

  public static boolean inStowPosition() {
    return getClimberPosition() <= STOW_ANGLE;
  }

  public static boolean inMountPosition() {
    return getClimberPosition() > MOUNT_ANGLE;
  }

  public static boolean inClimbPosition() {
    return getClimberPosition() <= CLIMB_ANGLE;
  }

  public static void unlockClimber() {
    s_latchServo.setAngle(SERVO_UNLOCK_ANGLE);
  }

  public static void lockClimber() {
    s_latchServo.setAngle(SERVO_LOCK_ANGLE);
  }

  public static void periodic() {
    var name = ClimbHardware.class.getSimpleName();
    Logger.recordOutput(
        name + "/absoluteEncoderValue",
        getClimberPosition()
    );
  }

}