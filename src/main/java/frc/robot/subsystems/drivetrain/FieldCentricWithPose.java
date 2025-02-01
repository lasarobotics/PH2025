package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class FieldCentricWithPose implements SwerveRequest {
    public double TargetX;
    public double TargetY;

    public double TargetXFeedForward = 0;
    public double TargetYFeedForward = 0;

    /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();
    /**
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     */
    public double TargetRateFeedforward = 0;

    /**
     * The allowable deadband of the request, in m/s.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     * For more information, see the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading.
     * Users can specify the PID gains to change how aggressively to maintain
     * heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second. Note that continuous input should
     * be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentric m_fieldCentric = new FieldCentric();
    
    public FieldCentricWithPose() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Rotation2d angleToFace = TargetDirection;
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /* If we're operator perspective, rotate the direction we want to face by the angle */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = TargetRateFeedforward +
            HeadingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                angleToFace.getRadians(),
                parameters.timestamp
            );

        double toApplyX = TargetXFeedForward +
            XController.calculate(
                parameters.currentPose.getX(),
                TargetX,
                parameters.timestamp
            );

        double toApplyY = TargetXFeedForward +
            YController.calculate(
                parameters.currentPose.getY(),
                TargetY,
                parameters.timestamp
            );

        return m_fieldCentric
            .withVelocityX(toApplyX)
            .withVelocityY(toApplyY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withCenterOfRotation(CenterOfRotation)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
            .withForwardPerspective(ForwardPerspective)
            .apply(parameters, modulesToApply);
    }

    public FieldCentricWithPose withTargetX(double x) {
        this.TargetX = x;
        return this;
    }

    public FieldCentricWithPose withTargetX(Distance x) {
        this.TargetX = x.in(Meters);
        return this;
    }

    public FieldCentricWithPose withTargetY(double y) {
        this.TargetY = y;
        return this;
    }

    public FieldCentricWithPose withTargetY(Distance y) {
        this.TargetY = y.in(Meters);
        return this;
    }

    public FieldCentricWithPose withTargetPose(Pose2d pose) {
        this.TargetX = pose.getMeasureX().in(Meters);
        this.TargetY = pose.getMeasureY().in(Meters);
        this.TargetDirection = pose.getRotation();
        return this;
    }

    public FieldCentricWithPose withFeedforwardX(double newVelocityX) {
        this.TargetXFeedForward = newVelocityX;
        return this;
    }

    public FieldCentricWithPose withFeedforwardX(LinearVelocity newVelocityX) {
        this.TargetXFeedForward = newVelocityX.in(MetersPerSecond);
        return this;
    }

    public FieldCentricWithPose withFeedforwardY(double newVelocityY) {
        this.TargetYFeedForward = newVelocityY;
        return this;
    }

    public FieldCentricWithPose withFeedforwardY(LinearVelocity newVelocityY) {
        this.TargetYFeedForward = newVelocityY.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the TargetDirection parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withTargetDirection(Rotation2d newTargetDirection) {
        this.TargetDirection = newTargetDirection;
        return this;
    }

    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withTargetRateFeedforward(double newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward;
        return this;
    }
    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public FieldCentricWithPose withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
        this.ForwardPerspective = newForwardPerspective;
        return this;
    }
}

