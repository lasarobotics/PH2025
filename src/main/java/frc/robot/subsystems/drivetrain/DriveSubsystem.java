package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;

import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;


import edu.wpi.first.math.geometry.Pose2d;

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
                if (s_shouldAutoAlign) return AUTO_ALIGN;
                return this;
            }
        },
        AUTO_ALIGN {
            long m_lastTime;
            TrapezoidProfile.State m_currentTurnState;
            TrapezoidProfile.State m_currentDriveXState;
            TrapezoidProfile.State m_currentDriveYState;

            @Override
            public void initialize() {
                m_lastTime = System.currentTimeMillis();

                m_currentTurnState = new TrapezoidProfile.State(
                    s_drivetrain.getState().Pose.getRotation().getRadians(),
                    s_drivetrain.getState().Speeds.omegaRadiansPerSecond
                );

                m_currentDriveXState = new TrapezoidProfile.State(
                    s_drivetrain.getState().Pose.getX(),
                    s_drivetrain.getState().Speeds.vxMetersPerSecond
                );

                m_currentDriveYState = new TrapezoidProfile.State(
                    s_drivetrain.getState().Pose.getY(),
                    s_drivetrain.getState().Speeds.vyMetersPerSecond
                );
            }

            @Override
            public void execute() {
                if (!s_shouldAutoAlign) {
                    System.err.println("s_shouldAutoAlign is false in auto align execute");
                    return;
                }
                double dt = (System.currentTimeMillis() - m_lastTime) / 1000.0;
                m_lastTime = System.currentTimeMillis();

                /*
                // if we're too far from the drivetrain target...
                if (Math.abs(m_currentDriveXState.position - s_drivetrain.getState().Pose.getX()) > 0.25
                    || Math.abs(m_currentDriveYState.position - s_drivetrain.getState().Pose.getY()) > 0.25
                    || Math.abs(m_currentTurnState.position - s_drivetrain.getState().Pose.getRotation().getRadians()) > 0.1
                ) {
                    // move the drivetrain target so the profile can correct for the error
                    m_currentDriveXState = new TrapezoidProfile.State(s_drivetrain.getState().Pose.getX(), s_drivetrain.getState().Speeds.vxMetersPerSecond);
                    m_currentDriveYState = new TrapezoidProfile.State(s_drivetrain.getState().Pose.getY(), s_drivetrain.getState().Speeds.vyMetersPerSecond);
                    m_currentTurnState = new TrapezoidProfile.State(s_drivetrain.getState().Pose.getRotation().getRadians(), s_drivetrain.getState().Speeds.omegaRadiansPerSecond);
                } */
                
                // Get error which is the smallest distance between goal and measurement
                double errorBound = Math.PI;
                double measurement = s_drivetrain.getState().Pose.getRotation().getRadians();
                double goalMinDistance =
                    MathUtil.inputModulus(s_autoAlignTargetTurn.position - measurement, -errorBound, errorBound);
                double setpointMinDistance =
                    MathUtil.inputModulus(s_autoAlignTargetTurn.position - measurement, -errorBound, errorBound);

                // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
                // may be outside the input range after this operation, but that's OK because the controller
                // will still go there and report an error of zero. In other words, the setpoint only needs to
                // be offset from the measurement by the input range modulus; they don't need to be equal.
                s_autoAlignTargetTurn.position = goalMinDistance + measurement;
                m_currentTurnState.position = setpointMinDistance + measurement;

                m_currentDriveXState = s_driveProfile.calculate(dt, m_currentDriveXState, s_autoAlignTargetDriveX);
                m_currentDriveYState = s_driveProfile.calculate(dt, m_currentDriveYState, s_autoAlignTargetDriveY);
                m_currentTurnState = s_turnProfile.calculate(dt, m_currentTurnState, s_autoAlignTargetTurn);
                
                s_drivetrain.setControl(
                    s_autoDrive
                    .withTargetDirection(new Rotation2d(m_currentTurnState.position))
                    .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
                    .withTargetX(m_currentDriveXState.position)
                    .withFeedforwardX(m_currentDriveXState.velocity)
                    .withTargetY(m_currentDriveYState.position)
                    .withFeedforwardY(m_currentDriveYState.velocity)
                );

                Logger.recordOutput("Drive/autoAlign/targetPose", new Pose2d(m_currentDriveXState.position, m_currentDriveYState.position, new Rotation2d(m_currentTurnState.position)));
                Logger.recordOutput("Drive/autoAlign/finalPose", new Pose2d(s_autoAlignTargetDriveX.position, s_autoAlignTargetDriveY.position, new Rotation2d(s_autoAlignTargetTurn.position)));
            }

            @Override
            public State nextState() {
                if (!s_shouldAutoAlign) {
                    return DRIVER_CONTROL;
                }

                if (Math.abs(s_drivetrain.getState().Pose.getRotation().getRadians() - s_autoAlignTargetTurn.position) < 0.05
                    && Math.abs(s_drivetrain.getState().Pose.getX() - s_autoAlignTargetDriveX.position) < 0.05
                    && Math.abs(s_drivetrain.getState().Pose.getY() - s_autoAlignTargetDriveY.position) < 0.05) {
                    return DRIVER_CONTROL;
                }
                return this;
            }

            @Override
            public void end(boolean interrupted) {
                s_shouldAutoAlign = false;
            }
        }
    }

    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static FieldCentricWithPose s_autoDrive;
    
    private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;

    private static boolean s_shouldAutoAlign = false;
    private static TrapezoidProfile.State s_autoAlignTargetDriveX;
    private static TrapezoidProfile.State s_autoAlignTargetDriveY;
    private static TrapezoidProfile.State s_autoAlignTargetTurn;

    private static ArrayList<AprilTagCamera> m_cameras;

    private static TrapezoidProfile s_turnProfile;
    private static TrapezoidProfile s_driveProfile;

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

        s_autoDrive = new FieldCentricWithPose()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0)
                .withRotationalDeadband(0);
        s_autoDrive.HeadingController.setPID(10, 0, 0);
        s_autoDrive.HeadingController.enableContinuousInput(0, Math.PI * 2);

        s_autoDrive.XController.setPID(5, 0, 0);
        s_autoDrive.YController.setPID(5, 0, 0);

        s_drivetrain.registerTelemetry(logger::telemeterize);

        s_turnProfile = new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS);
        s_driveProfile = new TrapezoidProfile(Constants.Drive.DRIVE_CONSTRAINTS);
    }

    public void bindControls(DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
        s_driveRequest = driveRequest;
        s_strafeRequest = strafeRequest;
        s_rotateRequest = rotateRequest;
    }

    /**
     * Request that the drivetrain aligns to the reef. Pass null to cancel.
     * @param state
     */
    public void requestAutoAlign(Pose2d pose) {
        s_autoAlignTargetDriveX = new TrapezoidProfile.State(pose.getX(), 0);
        s_autoAlignTargetDriveY = new TrapezoidProfile.State(pose.getY(), 0);
        s_autoAlignTargetTurn = new TrapezoidProfile.State(pose.getRotation().getRadians(), 0);
        s_shouldAutoAlign = true;
    }

    public void requestAutoAlign() {
        requestAutoAlign(findAutoAlignTarget());
    }

    private Pose2d findAutoAlignTarget() {
        double angle = Math.toDegrees(Math.atan2(s_drivetrain.getState().Pose.getX() - Constants.Field.REEF_LOCATION.getX(), s_drivetrain.getState().Pose.getY() - Constants.Field.REEF_LOCATION.getY())) + 360;
        angle = (((angle / 30)) + 10) % 12;
        Logger.recordOutput("Drive/snappedAngle", angle);
        return Constants.Drive.AUTO_ALIGN_LOCATIONS.get((int) angle);
        // return Constants.Drive.AUTO_ALIGN_LOCATIONS.get(0);
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
      rearCamera
    );
    return driveHardware;
  }

  /**
   * Get's the robot current pose
   * @return Robot's current pose
   */
  public Pose2d getPose(){
    return s_drivetrain.getState().Pose;
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

        Logger.recordOutput("Drive/state", getState().toString());
    }

    @Override
    public void close() throws Exception {
        s_drivetrain.close();
    }
}
