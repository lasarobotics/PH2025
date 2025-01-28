package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.vision.AprilTagCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
    public static record Hardware(
            AprilTagCamera frontLeftCamera,
            AprilTagCamera frontRightCamera,
            AprilTagCamera rearCamera) {
    }

    public enum State implements SystemState {
        DRIVER_CONTROL {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                s_drivetrain.setControl(s_drive
                    .withVelocityX(Constants.Drive.MAX_SPEED.times(-s_driveRequest.getAsDouble()))
                    .withVelocityY(Constants.Drive.MAX_SPEED.times(-s_strafeRequest.getAsDouble()))
                    .withRotationalRate(Constants.Drive.MAX_ANGULAR_RATE.times(-s_rotateRequest.getAsDouble()))
                );
            }

            @Override
            public State nextState() {
                if (s_autoAlignTargetState.isPresent())
                    return AUTO_ALIGN;
                return this;
            }
        },
        AUTO_ALIGN {
            long m_lastTime;
            TrapezoidProfile.State m_currentState;

            @Override
            public void initialize() {
                m_lastTime = System.currentTimeMillis();
            }

            @Override
            public void execute() {
                long dt = System.currentTimeMillis() - m_lastTime;
                m_lastTime = System.currentTimeMillis();
                m_currentState = new TrapezoidProfile.State(s_drivetrain.getRotation3d().getZ(), 0);

                TrapezoidProfile.State newState = s_turnProfile.calculate(dt / 1000.0, m_currentState, s_autoAlignTargetState.orElseThrow());
                // System.out.println("new state: " + newState.position + ", " + newState.velocity);
                
                
                s_drivetrain.setControl(
                    s_autoDrive
                    .withRotationalDeadband(0)
                    .withTargetDirection(new Rotation2d(3.14159))
                );
            }

            @Override
            public State nextState() {
                if (Math.abs(m_currentState.position - s_autoAlignTargetState.orElseThrow().position) < 0.05) {
                    return DRIVER_CONTROL;
                }
                return this;
            }

            @Override
            public void end(boolean interrupted) {
                s_autoAlignTargetState = Optional.empty();
            }
        }
    }

    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static SwerveRequest.FieldCentricFacingAngle s_autoDrive;
    private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;

    private static Optional<TrapezoidProfile.State> s_autoAlignTargetState = Optional.empty();

    private static Telemetry s_logger;

    private static ArrayList<AprilTagCamera> m_cameras;

    private static TrapezoidProfile s_turnProfile;

    private static SystemState s_lastState;

    public DriveSubsystem(Hardware driveHardware, Telemetry logger) {
        super(State.DRIVER_CONTROL);

        m_cameras = new ArrayList<AprilTagCamera>();
        m_cameras.add(driveHardware.frontLeftCamera);
        m_cameras.add(driveHardware.frontRightCamera);
        m_cameras.add(driveHardware.rearCamera);

        s_drivetrain = TunerConstants.createDrivetrain();

        /* Setting up bindings for necessary control of the swerve drive platform */
        s_drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.Drive.MAX_SPEED.times(0.1))
                .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        s_autoDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0)
                .withRotationalDeadband(0);
        s_autoDrive.HeadingController.setPID(1, 0, 0);

        s_autoDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDeadband(0)
            .withRotationalDeadband(0);
        s_autoDrive.HeadingController.setPID(1, 0, 0);

        s_logger = logger;
        s_drivetrain.registerTelemetry(logger::telemeterize);

        s_turnProfile = new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS);
    }

    public void bindControls(DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
        s_driveRequest = driveRequest;
        s_strafeRequest = strafeRequest;
        s_rotateRequest = rotateRequest;
    }

    public void requestAutoAlign(TrapezoidProfile.State state) {
        s_autoAlignTargetState = Optional.of(state);
        s_lastState = getState();
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
                Constants.Field.FIELD_LAYOUT);

        AprilTagCamera frontRightCamera = new AprilTagCamera(
                Constants.VisionHardware.CAMERA_B_NAME,
                Constants.VisionHardware.CAMERA_B_LOCATION,
                Constants.VisionHardware.CAMERA_B_RESOLUTION,
                Constants.VisionHardware.CAMERA_B_FOV,
                Constants.Field.FIELD_LAYOUT);

        AprilTagCamera rearCamera = new AprilTagCamera(
                Constants.VisionHardware.CAMERA_C_NAME,
                Constants.VisionHardware.CAMERA_C_LOCATION,
                Constants.VisionHardware.CAMERA_C_RESOLUTION,
                Constants.VisionHardware.CAMERA_C_FOV,
                Constants.Field.FIELD_LAYOUT);

        Hardware driveHardware = new Hardware(
                frontLeftCamera,
                frontRightCamera,
                rearCamera);
        return driveHardware;
    }

    @Override
    public void periodic() {
        // Add AprilTag pose estimates if available
        for (var camera : m_cameras) {
            var result = camera.getLatestEstimatedPose();

            // If no updated vision pose estimate, continue
            if (result == null)
                continue;
            // Add vision measurement
            s_drivetrain.addVisionMeasurement(
                    result.estimatedRobotPose.estimatedPose.toPose2d(),
                    result.estimatedRobotPose.timestampSeconds,
                    result.standardDeviation);
        }
    }

    @Override
    public void close() throws Exception {
        s_drivetrain.close();
    }
}
