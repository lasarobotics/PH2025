package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.RobotContainer;
import frc.robot.lib.State;
import frc.robot.lib.StateMachine;

public class DriveSubsystem extends StateMachine {

  private static final SwerveRequest.FieldCentric s_drive =
    new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
      .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
  
  private static final SwerveRequest.RobotCentric s_driveRobotCentric =
    new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withDeadband(0)
      .withRotationalDeadband(0);
  
  private static final FieldCentricWithPose s_autoDrive =
    new FieldCentricWithPose()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static boolean s_shouldAutoAlign = false;
  private static Pose2d s_autoAlignTarget = new Pose2d();
  private static TrapezoidProfile.State s_autoAlignTargetDriveX;
  private static TrapezoidProfile.State s_autoAlignTargetDriveY;
  private static TrapezoidProfile.State s_autoAlignTargetTurn;

  private static final TrapezoidProfile s_turnProfile =
    new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS);

  private static final TrapezoidProfile s_driveProfile =
    new TrapezoidProfile(Constants.Drive.DRIVE_CONSTRAINTS);

  private static final Double DEADBAND_SCALAR = 0.085;

  /** Robot is within the auto align tolerance of the target point */
  private static boolean s_isAligned;

  /** Robot is within auto align tolerance * 2 of the target point */
  private static boolean s_isClose;

  private static double s_driveSpeedScalar = Constants.Drive.FAST_SPEED_SCALAR;
  
  private static DriveSubsystem s_instance;

  private DriveSubsystem() {
    super(DrivetrainState.DRIVER_CONTROL);

    s_autoDrive.HeadingController.setPID(5, 0, 0);
    s_autoDrive.HeadingController.enableContinuousInput(0, Math.PI * 2);

    s_autoDrive.XController.setPID(6, 0, 0);
    s_autoDrive.YController.setPID(6, 0, 0);
  }

  public static DriveSubsystem getInstance() {
    if (DriveSubsystem.s_instance == null) {
      DriveSubsystem.s_instance = new DriveSubsystem();
    }
    return DriveSubsystem.s_instance;
  }

  public enum DrivetrainState implements State {
    AUTO {
      @Override
      public State nextState() {
        if (s_shouldAutoAlign) return AUTO_ALIGN;
        if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;
        return this;
      }
    },
    DRIVER_CONTROL {
      @Override
      public void execute() {
        DriveHardware.s_drivetrain.setControl(
          s_drive
            .withVelocityX(
              Constants.Drive.MAX_SPEED
                .times(-s_strafeRequest.getAsDouble())
                .times(s_driveSpeedScalar)
            )
            .withVelocityY(
              Constants.Drive.MAX_SPEED
                .times(-s_driveRequest.getAsDouble())
                .times(s_driveSpeedScalar)
            )
            .withRotationalRate(
              Constants.Drive.MAX_ANGULAR_RATE
                .times(-s_rotateRequest.getAsDouble())
                .times(s_driveSpeedScalar)
            )
        );
      }

      @Override
      public DrivetrainState nextState() {
        if (DriverStation.isAutonomous()) return AUTO;
        if (
          s_shouldAutoAlign &&
          Math.abs(s_strafeRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR &&
          Math.abs(s_driveRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR &&
          Math.abs(s_rotateRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR
        ) {
          requestAutoAlign();
          return AUTO_ALIGN;
        }
        return this;
      }
    },
    AUTO_ALIGN {
      long m_lastTime;
      long m_closeTime;

      TrapezoidProfile.State m_currentTurnState;
      TrapezoidProfile.State m_currentDriveXState;
      TrapezoidProfile.State m_currentDriveYState;

      boolean secondStage = false;
      boolean thirdStage = false;
      Timer timer = new Timer();

      /**
       * Set the current motion profile state to the actual state of the robot
       */
      private void resetMotionProfile() {
        var drivetrain_state = DriveHardware.s_drivetrain.getState();
        var pose = drivetrain_state.Pose.rotateAround(
          s_autoAlignTarget.getTranslation(),
          s_autoAlignTarget.getRotation().times(-1)
        );
        var field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain_state.Speeds,
          drivetrain_state.Pose.getRotation()
        );
        var field_speeds_pose = new Translation2d(
          field_speeds.vxMetersPerSecond,
          field_speeds.vyMetersPerSecond
        )
          .rotateBy(s_autoAlignTarget.getRotation().times(-1));

        m_currentTurnState = new TrapezoidProfile.State(
          drivetrain_state.Pose.getRotation().getRadians(),
          field_speeds.omegaRadiansPerSecond
        );

        m_currentDriveXState = new TrapezoidProfile.State(
          pose.getX(),
          field_speeds_pose.getX()
        );

        m_currentDriveYState = new TrapezoidProfile.State(
          pose.getY(),
          field_speeds_pose.getY()
        );
      }

      @Override
      public void initialize() {
        m_lastTime = System.currentTimeMillis();
        m_closeTime = System.currentTimeMillis();

        secondStage = false;
        thirdStage = false;
        // move auto align away from the reef slightly
        // s_autoAlignTarget = s_autoAlignTarget.plus(new Transform2d(new Translation2d(-0.3, 0), new Rotation2d()));

        s_autoAlignTargetDriveX.position = s_autoAlignTarget.getX();
        s_autoAlignTargetDriveY.position = s_autoAlignTarget.getY();
        
        var drivetrain_state = DriveHardware.s_drivetrain.getState();
        var pose = drivetrain_state.Pose.rotateAround(s_autoAlignTarget.getTranslation(), s_autoAlignTarget.getRotation().times(-1));
        var field_speeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain_state.Speeds, drivetrain_state.Pose.getRotation());
        var field_speeds_pose = new Translation2d(field_speeds.vxMetersPerSecond, field_speeds.vyMetersPerSecond).rotateBy(s_autoAlignTarget.getRotation().times(-1));

        m_currentTurnState =
            new TrapezoidProfile.State(
                drivetrain_state.Pose.getRotation().getRadians(),
                field_speeds.omegaRadiansPerSecond);

        m_currentDriveXState =
            new TrapezoidProfile.State(
                pose.getX(), field_speeds_pose.getX());

        m_currentDriveYState =
            new TrapezoidProfile.State(
                pose.getY(), field_speeds_pose.getY());

        s_isAligned = false;
        timer.restart();
      }

      @Override
      public void execute() {
        double dt = (System.currentTimeMillis() - m_lastTime) / 1000.0;
        m_lastTime = System.currentTimeMillis();
        Logger.recordOutput("Drive/dt", dt);

        // Get error which is the smallest distance between goal and measurement
        double errorBound = Math.PI;
        double measurement = DriveHardware.s_drivetrain.getState().Pose.getRotation().getRadians();
        double goalMinDistance =
            MathUtil.inputModulus(
                s_autoAlignTargetTurn.position - measurement, -errorBound, errorBound);
        double setpointMinDistance =
            MathUtil.inputModulus(
                m_currentTurnState.position - measurement, -errorBound, errorBound);

        // Recompute the profile goal with the smallest error, thus giving the shortest
        // path. The goal
        // may be outside the input range after this operation, but that's OK because
        // the controller
        // will still go there and report an error of zero. In other words, the setpoint
        // only needs to
        // be offset from the measurement by the input range modulus; they don't need to
        // be equal.
        s_autoAlignTargetTurn.position = goalMinDistance + measurement;
        m_currentTurnState.position = setpointMinDistance + measurement;

        m_currentDriveXState =
            s_driveProfile.calculate(dt, m_currentDriveXState, s_autoAlignTargetDriveX);
        m_currentDriveYState =
            s_driveProfile.calculate(dt, m_currentDriveYState, s_autoAlignTargetDriveY);
        m_currentTurnState = s_turnProfile.calculate(dt, m_currentTurnState, s_autoAlignTargetTurn);

        Translation2d newPosition = new Translation2d(m_currentDriveXState.position, m_currentDriveYState.position).rotateAround(s_autoAlignTarget.getTranslation(), s_autoAlignTarget.getRotation().times(1));
        Translation2d newVelocity = new Translation2d(m_currentDriveXState.velocity, m_currentDriveYState.velocity).rotateBy(s_autoAlignTarget.getRotation().times(1));

        var drivetrain_state = DriveHardware.s_drivetrain.getState();
        var drivetrain_pose = drivetrain_state.Pose;
        double distance =
            drivetrain_pose.getTranslation().getDistance(s_autoAlignTarget.getTranslation());
        double heading =
            Math.abs((drivetrain_pose.getRotation().getRadians() - s_autoAlignTargetTurn.position))
                % 360;

        var perp_dist =
            Math.cos(s_autoAlignTarget.getRotation().getRadians())
                    * (s_autoAlignTarget.getY() - drivetrain_pose.getY())
                - Math.sin(s_autoAlignTarget.getRotation().getRadians())
                    * (s_autoAlignTarget.getX() - drivetrain_pose.getX());
        // var perp_dist = DriveHardware.s_drivetrain.getState().Pose.getY() - m_currentDriveYState.position;
        Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/DistanceToScoreLine", perp_dist);

        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
                || heading > (Math.PI * 2 - (Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN)))) {
          thirdStage = true;
        }

        // if the robot is _very_ close to the target, turn off the drivetrain
        if ((distance < Constants.Drive.AUTO_ALIGN_TOLERANCE || thirdStage)
            && Math.abs(perp_dist) < Constants.Drive.AUTO_ALIGN_LR_TOLERANCE
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
                || heading > (Math.PI * 2 - (Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN)))) {
          DriveHardware.s_drivetrain.setControl(
              s_driveRobotCentric.withVelocityX(0).withVelocityY(0.0).withRotationalRate(0));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", true);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "deadband");
            s_isAligned = DriverStation.isAutonomous() ? timer.hasElapsed(0.01) : DriveHardware.seesTag() && timer.hasElapsed(0.1);
        } else if (thirdStage) {
          DriveHardware.s_drivetrain.setControl(
              s_driveRobotCentric
                  .withVelocityX(0.0)
                  .withDeadband(0.0)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityY(MathUtil.clamp(perp_dist * 20, -0.05 * Drive.MAX_SPEED.in(MetersPerSecond), 0.05 * Drive.MAX_SPEED.in(MetersPerSecond)))
                  .withRotationalRate(0));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", false);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "lr");
          s_isAligned = false;
          timer.restart();
        } else {
          DriveHardware.s_drivetrain.setControl(
              s_autoDrive
                  .withTargetDirection(new Rotation2d(m_currentTurnState.position))
                  .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
                  .withTargetX(newPosition.getX())
                  .withFeedforwardX(newVelocity.getX())
                  .withTargetY(newPosition.getY())
                  .withFeedforwardY(newVelocity.getY()));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", false);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "normal");
          s_isAligned = false;
          timer.restart();
        }

        Logger.recordOutput(
            "DriveSubsystem/autoAlign/targetPose",
            new Pose2d(
                newPosition,
                new Rotation2d(m_currentTurnState.position)));
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/finalPose",
            new Pose2d(
                s_autoAlignTargetDriveX.position,
                s_autoAlignTargetDriveY.position,
                new Rotation2d(s_autoAlignTargetTurn.position)));
        Logger.recordOutput("DriveSubsystem/autoAlign/finalPose2", s_autoAlignTarget);

        Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/distanceError", distance);
        Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/headingError", heading);
        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
                || heading > (Math.PI * 2 - Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN))) {
          secondStage = true;
          thirdStage = true;

          resetMotionProfile();
        }

        Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/secondStage", secondStage);

        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE * 2
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN * 2
                || heading > (Math.PI * 2 - Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN * 2))) {
          s_isClose = true;
        } else {
          s_isClose = false;
        }

        if (!s_isClose) {
          m_closeTime = System.currentTimeMillis();
        }

        Logger.recordOutput("DriveSubsystem/autoAlign/isClose", s_isClose);
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/closeTime", System.currentTimeMillis() - m_closeTime);

        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/x",
            DriveHardware.s_drivetrain.getState().Pose.getX() - m_currentDriveXState.position);
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/y",
            DriveHardware.s_drivetrain.getState().Pose.getY() - m_currentDriveYState.position);
      }

      @Override
      public DrivetrainState nextState() {
        if (!s_shouldAutoAlign) {
          if (DriverStation.isAutonomous()) return AUTO;
          else return DRIVER_CONTROL;
        }

        if (
          Math.abs(s_strafeRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR ||
          Math.abs(s_driveRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR ||
          Math.abs(s_rotateRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR
        ) {
          if (!DriverStation.isAutonomous()) {
            return DRIVER_CONTROL;
          }
        }
        return this;
      }

      @Override
      public void end(State nextState) {
        s_isAligned = false;

        DriveHardware.s_drivetrain.setControl(s_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
    }
  }

  /*
   * Binds the controls needed to drive for controller usage
   */
  public void bindControls(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
  }

  /**
   * Request an auto align to the pole nearest to the pose passed in
   * @param pose Pose to pass in which the robot aligns to the pole nearest to that pose
   */
  public static void requestAutoAlign(Pose2d pose) {
    Logger.recordOutput("temp/requestedPose", pose);
    s_autoAlignTarget = pose;
    s_autoAlignTargetDriveX = new TrapezoidProfile.State(pose.getX(), 0);
    s_autoAlignTargetDriveY = new TrapezoidProfile.State(pose.getY(), 0);
    s_autoAlignTargetTurn = new TrapezoidProfile.State(pose.getRotation().getRadians(), 0);
    s_shouldAutoAlign = true;
    Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/shouldAutoAlign", s_shouldAutoAlign);
  }

  /*
   * Requests an auto align based on the pole that is nearest to the robot's pose
   */
  public static void requestAutoAlign() {
    requestAutoAlign(findAutoAlignTarget());
  }

  /**
   * Cancels the auto align of the robot
   */
  public void cancelAutoAlign() {
    s_shouldAutoAlign = false;
  }

  /**
   * Checks if the robot is near the source
   * @return True if robot is near source
   */
  public boolean isNearSource() {
    var robotPose = DriveHardware.s_drivetrain.getState().Pose;
    return 
      robotPose.getX() < 2.0 &&
      (
        robotPose.getY() < 1.5 ||
        robotPose.getY() > 6.0
      );
  }

  /**
   * Check whether the robot is aligned to the reef to score or not
   * @return A boolean showing whether the robot is aligned to the reef or not
   */
  public boolean isAligned() {
    return s_isAligned;
  }

  /**
   * Finds the nearest reef pole to align to from the robot's pose
   * @return A pose closest aligned to the nearest reef pole from the robot's current pose
   */
  private static Pose2d findAutoAlignTarget() {
    return findAutoAlignTarget(DriveHardware.s_drivetrain.getState().Pose);
  }

  /**
   * Finds the nearest auto align target to align to from a passed in pose
   * @param startPose The pose passed in which the robot finds the aligned pose nearest to
   */
  public static Pose2d findAutoAlignTarget(Pose2d startPose) {
    var pose = startPose.nearest(findAutoAlignTargets());
    Logger.recordOutput("temp/foundPose", pose);
    return pose;
  }

  /**
   * Returns the location the robot should go to in order to align to the nearest reef pole
   * flipSide will cause the robot to align to the farther pole on the same side of the reef.
  */
  private static List<Pose2d> findAutoAlignTargets() {
    Translation2d reefLocation;

    // Determine which reef we're aligning to
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
      reefLocation = Constants.Field.REEF_LOCATION_RED;
    } else {
      reefLocation = Constants.Field.REEF_LOCATION_BLUE;
    }

    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/reefLocation",
        new Pose2d(reefLocation, Rotation2d.fromDegrees(0)));


    Pose2d left_pose;
    Pose2d right_pose;

    ArrayList<Pose2d> branch_locations = new ArrayList<>();
    for (int angle = 0; angle < 360; angle += 60) {
      Translation2d left_offset = Constants.Drive.LEFT_BRANCH_OFFSET;
      Translation2d right_offset = Constants.Drive.RIGHT_BRANCH_OFFSET;
      left_offset = left_offset.rotateBy(Rotation2d.fromDegrees(angle));
      right_offset = right_offset.rotateBy(Rotation2d.fromDegrees(angle));
      left_pose = new Pose2d(reefLocation.plus(left_offset), Rotation2d.fromDegrees(angle));
      right_pose = new Pose2d(reefLocation.plus(right_offset), Rotation2d.fromDegrees(angle));
      branch_locations.add(left_pose);
      branch_locations.add(right_pose);
    }

    return branch_locations;
  }

  /**
   * Sets the drive speed of the robot
   * @param newSpeed speed to set the robot to
   */
  public void setDriveSpeed(double newSpeed) {
    s_driveSpeedScalar = newSpeed;
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/autoAlign/autotarget", findAutoAlignTarget());
    Logger.recordOutput(getName() + "/isNearSource", isNearSource());
    Logger.recordOutput(getName() + "/autoAlign/isAligned", s_isAligned);
  }
}