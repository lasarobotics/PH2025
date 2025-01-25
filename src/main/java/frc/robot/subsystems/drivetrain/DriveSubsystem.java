package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.vision.AprilTagCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware(
    AprilTagCamera frontLeftCamera,
    AprilTagCamera frontRightCamera,
    AprilTagCamera rearCamera
  ) {}

  public enum State implements SystemState {
    DRIVER_CONTROL {
      @Override
      public void initialize() {
      }

      @Override
      public void execute() {
        s_drivetrain.setControl(
          s_drive.withVelocityX(Constants.Drive.MAX_SPEED.times(-s_driveRequest.getAsDouble())) // Drive forward with negative Y (forward)
            .withVelocityY(Constants.Drive.MAX_SPEED.times(-s_strafeRequest.getAsDouble())) // Drive left with negative X (left)
            .withRotationalRate(Constants.Drive.MAX_ANGULAR_RATE.times(-s_rotateRequest.getAsDouble())) // Drive counterclockwise with negative X (left)
        );
      }

      @Override
      public State nextState() {
        return this; // no escape
      }
    },
  }

  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static Telemetry s_logger;

  public DriveSubsystem(Hardware driveHardware, Telemetry logger) {
    super(State.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    s_drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drive.MAX_SPEED.times(0.1))
      .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    s_logger = logger;
    s_drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Initialize hardware devices for drive subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    AprilTagCamera frontLeftCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_A_NAME,
      Constants.VisionHardware.CAMERA_A_LOCATION,
      Constants.VisionHardware.CAMERA_A_RESOLUTION,
      Constants.VisionHardware.CAMERA_A_FOV,
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    );

    AprilTagCamera frontRightCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_B_NAME,
      Constants.VisionHardware.CAMERA_B_LOCATION,
      Constants.VisionHardware.CAMERA_B_RESOLUTION,
      Constants.VisionHardware.CAMERA_B_FOV,
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    );

    AprilTagCamera rearCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_C_NAME,
      Constants.VisionHardware.CAMERA_C_LOCATION,
      Constants.VisionHardware.CAMERA_C_RESOLUTION,
      Constants.VisionHardware.CAMERA_C_FOV,
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    );

    Hardware driveHardware = new Hardware(
      frontLeftCamera,
      frontRightCamera,
      rearCamera
    );
    return driveHardware;
  }

  public void bindControls(DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
  }

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }
}
