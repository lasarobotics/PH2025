package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SystemState;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
    public static record Hardware() {
    }

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
