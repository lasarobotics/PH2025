package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LoopTimer;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
// import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware() {}

  public enum State implements SystemState {
    NOTHING {

      @Override
      public SystemState nextState() {
        return this;
      }
    },
    AUTO {
      @Override
      public SystemState nextState() {
        if (s_shouldAutoAlign) return AUTO_ALIGN;
        if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;
        return this;
      }
    },
    DRIVER_CONTROL {
      @Override
      public void execute() {
        s_drivetrain.setControl(
            s_drive
                .withVelocityX(
                    Constants.Drive.MAX_SPEED
                        .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                        .times(s_driveSpeedScalar))
                .withVelocityY(
                    Constants.Drive.MAX_SPEED
                        .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                        .times(s_driveSpeedScalar))
                .withRotationalRate(
                    Constants.Drive.MAX_ANGULAR_RATE.times(-s_rotateRequest.getAsDouble())));
      }

      @Override
      public State nextState() {
        if (s_shouldAutoAlign
            && Math.abs(s_strafeRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR
            && Math.abs(s_driveRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR
            && Math.abs(s_rotateRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR) {
          requestAutoAlign();
          return AUTO_ALIGN;
        }
        if (DriverStation.isAutonomous()) return AUTO;
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

      @Override
      public void initialize() {
        m_lastTime = System.currentTimeMillis();
        m_closeTime = System.currentTimeMillis();

        secondStage = false;
        // move auto align 0.1 meters away from the reef
        s_autoAlignTarget = s_autoAlignTarget.plus(new Transform2d(new Translation2d(-0.2, 0), new Rotation2d()));
        s_autoAlignTargetDriveX.position -= 0.2;

        // make sure the motion profiles are at normal speed
        s_turnProfile = new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS);
        s_driveProfile = new TrapezoidProfile(Constants.Drive.DRIVE_CONSTRAINTS);
        
        var drivetrain_state = s_drivetrain.getState();
        var field_speeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain_state.Speeds, drivetrain_state.Pose.getRotation());

        m_currentTurnState =
            new TrapezoidProfile.State(
                drivetrain_state.Pose.getRotation().getRadians(),
                field_speeds.omegaRadiansPerSecond);

        m_currentDriveXState =
            new TrapezoidProfile.State(
                drivetrain_state.Pose.getX(), field_speeds.vxMetersPerSecond);

        m_currentDriveYState =
            new TrapezoidProfile.State(
                drivetrain_state.Pose.getY(), field_speeds.vyMetersPerSecond);

        s_isAligned = false;
      }

      @Override
      public void execute() {
        double dt = (System.currentTimeMillis() - m_lastTime) / 1000.0;
        m_lastTime = System.currentTimeMillis();
        Logger.recordOutput("Drive/dt", dt);

        // Get error which is the smallest distance between goal and measurement
        double errorBound = Math.PI;
        double measurement = s_drivetrain.getState().Pose.getRotation().getRadians();
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

        var drivetrain_state = s_drivetrain.getState();
        var drivetrain_pose = drivetrain_state.Pose;
        double distance =
            drivetrain_pose.getTranslation().getDistance(s_autoAlignTarget.getTranslation());
        double heading =
            Math.abs((drivetrain_pose.getRotation().getRadians() - s_autoAlignTargetTurn.position))
                % 360;

        // Translation2d newPosition = new Translation2d(m_currentDriveXState.position,
        // m_currentDriveYState.position).rotateAround(s_autoAlignTarget.getTranslation(),
        // s_autoAlignTarget.getRotation().times(1));
        // Translation2d newVelocity = new Translation2d(m_currentDriveXState.velocity,
        // m_currentDriveYState.velocity).rotateBy(s_autoAlignTarget.getRotation().times(1));

        var perp_dist =
            Math.cos(s_autoAlignTarget.getRotation().getRadians())
                    * (s_autoAlignTarget.getY() - drivetrain_pose.getY())
                - Math.sin(s_autoAlignTarget.getRotation().getRadians())
                    * (s_autoAlignTarget.getX() - drivetrain_pose.getX());
        Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/DistanceToScoreLine", perp_dist);

        // if the robot is _very_ close to the target, turn off the drivetrain
        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE
            && Math.abs(perp_dist) < Constants.Drive.AUTO_ALIGN_LR_TOLERANCE
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
                || heading > (Math.PI * 2 - (Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN)))) {
          s_drivetrain.setControl(
              s_driveRobotCentric.withVelocityX(0).withVelocityY(0.0).withRotationalRate(0));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", true);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "deadband");
        } else if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
                || heading > (Math.PI * 2 - (Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN)))) {
          s_drivetrain.setControl(
              s_driveRobotCentric
                  .withVelocityX(0.0)
                  .withDeadband(0.0)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityY(perp_dist * 2)
                  .withRotationalRate(0));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", false);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "lr");
        } else {
          s_drivetrain.setControl(
              s_autoDrive
                  .withTargetDirection(new Rotation2d(m_currentTurnState.position))
                  .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
                  .withTargetX(m_currentDriveXState.position)
                  .withFeedforwardX(m_currentDriveXState.velocity)
                  .withTargetY(m_currentDriveYState.position)
                  .withFeedforwardY(m_currentDriveYState.velocity));
          Logger.recordOutput(
              RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", false);
          Logger.recordOutput(
            RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/controlMode", "normal");
        }

        Logger.recordOutput(
            "DriveSubsystem/autoAlign/targetPose",
            new Pose2d(
                m_currentDriveXState.position,
                m_currentDriveYState.position,
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
          if (secondStage) {
            s_isAligned = true;
          } else {
            secondStage = true;

            // move the target back to normal, and lower the constraints
            s_turnProfile = new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS_SLOW);
            s_driveProfile = new TrapezoidProfile(Constants.Drive.DRIVE_CONSTRAINTS_SLOW);
            requestAutoAlign();
          }
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
        // if (System.currentTimeMillis() - m_closeTime > 300) {
        //   s_isAligned = true;
        // }

        Logger.recordOutput("DriveSubsystem/autoAlign/isClose", s_isClose);
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/closeTime", System.currentTimeMillis() - m_closeTime);

        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/x",
            s_drivetrain.getState().Pose.getX() - m_currentDriveXState.position);
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/y",
            s_drivetrain.getState().Pose.getY() - m_currentDriveYState.position);
      }

      @Override
      public State nextState() {
        if (!s_shouldAutoAlign) {
          if (DriverStation.isAutonomous()) return AUTO;
          else return DRIVER_CONTROL;
        }

        if (Math.abs(s_strafeRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR
            || Math.abs(s_driveRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR
            || Math.abs(s_rotateRequest.getAsDouble()) > DriveSubsystem.DEADBAND_SCALAR) {
          return DRIVER_CONTROL;
        }
        // if (s_isAligned && DriverStation.isTeleop()) {
        //   return DRIVER_CONTROL;
        // }
        return this;
      }

      @Override
      public void end(boolean interrupted) {
        // s_shouldAutoAlign = false;
        s_isAligned = false;
      }
    }
  }

  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  private static SwerveRequest.RobotCentric s_driveRobotCentric;
  private static FieldCentricWithPose s_autoDrive;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static boolean s_shouldAutoAlign = false;
  private static Pose2d s_autoAlignTarget = new Pose2d();
  private static TrapezoidProfile.State s_autoAlignTargetDriveX;
  private static TrapezoidProfile.State s_autoAlignTargetDriveY;
  private static TrapezoidProfile.State s_autoAlignTargetTurn;

  private static TrapezoidProfile s_turnProfile;
  private static TrapezoidProfile s_driveProfile;
  private static final Double DEADBAND_SCALAR = 0.05;

  /** Robot is within the auto align tolerance of the target point */
  private static boolean s_isAligned;

  /** Robot is within auto align tolerance * 2 of the target point */
  private static boolean s_isClose;

  // private static ArrayList<AprilTagCamera> m_cameras;

  private static double s_driveSpeedScalar = 0.2;

  protected final Thread m_limelight_thread;

  public DriveSubsystem(Hardware driveHardware, Telemetry logger) {
    super(State.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();
    /* Setting up bindings for necessary control of the swerve drive platform */
    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    s_driveRobotCentric =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    s_autoDrive =
        new FieldCentricWithPose()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    s_autoDrive.HeadingController.setPID(5, 0, 0);
    s_autoDrive.HeadingController.enableContinuousInput(0, Math.PI * 2);

    s_autoDrive.XController.setPID(1.5, 0, 0);
    s_autoDrive.YController.setPID(4, 0, 0);

    s_drivetrain.registerTelemetry(logger::telemeterize);

    s_turnProfile = new TrapezoidProfile(Constants.Drive.TURN_CONSTRAINTS);
    s_driveProfile = new TrapezoidProfile(Constants.Drive.DRIVE_CONSTRAINTS);

    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
  }

  public void limelight_thread_func() {

    String[] limelights = {"limelight-left", "limelight-right"};

    while (true) {
      for (String limelight : limelights) {
        LimelightHelpers.SetIMUMode(limelight, DriverStation.isDisabled() ? 1 : 2);
        LimelightHelpers.setLimelightNTDouble(
            limelight, "throttle_set", DriverStation.isDisabled() ? 100 : 0);
        LimelightHelpers.SetRobotOrientation(
            limelight, s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        Logger.recordOutput(
            getName() + "/" + limelight + "/botpose",
            LimelightHelpers.getBotPose3d_wpiBlue(limelight));
        double[] poseEntry =
            LimelightHelpers.getLimelightNTDoubleArray(limelight, "botpose_orb_wpiblue");
        Logger.recordOutput(
            getName() + "/" + limelight + "/botpose_orb", LimelightHelpers.toPose3D(poseEntry));
        LimelightHelpers.PoseEstimate pose_estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

        if (pose_estimate == null) continue;
        boolean doRejectUpdate = false;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
          int[] validIds = {6, 7, 8, 9, 10, 11};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        } else {
          int[] validIds = {17, 18, 19, 20, 21, 22};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        }
        if (s_drivetrain.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI) {
          doRejectUpdate = true;
        }

        if (pose_estimate.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          s_drivetrain.addVisionMeasurement(
              pose_estimate.pose, Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds));
          // Logger.recordOutput(getName() + "/" + limelight + "/botpose_orb", pose_estimate.pose);
        }
      }
      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        // e.printStackTrace();
      }
    }
  }

  public void bindControls(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
  }

  /**
   * Request that the drivetrain aligns to the reef
   *
   * @param state
   */
  public static void requestAutoAlign(Pose2d pose) {
    s_autoAlignTarget = pose;
    s_autoAlignTargetDriveX = new TrapezoidProfile.State(pose.getX(), 0);
    s_autoAlignTargetDriveY = new TrapezoidProfile.State(pose.getY(), 0);
    s_autoAlignTargetTurn = new TrapezoidProfile.State(pose.getRotation().getRadians(), 0);
    s_shouldAutoAlign = true;
  }

  public static void requestAutoAlign() {
    requestAutoAlign(findAutoAlignTarget());
  }

  public void cancelAutoAlign() {
    s_shouldAutoAlign = false;
  }

  public Pose2d getPose() {
    return s_drivetrain.getState().Pose;
  }

  /**
   * Checks if the robot is near the source
   *
   * @return True if robot is near source
   */
  public boolean isNearSource() {
    return ((getPose().getX() < 2.0) && ((getPose().getY() < 1.5) || getPose().getY() > 6.0));
  }

  public boolean isAligned() {
    return s_isAligned;
  }

  public void resetPose() {
    s_drivetrain.resetPose();
  }

  public void resetPose(Pose2d pose) {
    s_drivetrain.resetPose(pose);
  }

  /**
   * Returns the location the robot should go to in order to align to the nearest reef pole
   * flipSide will cause the robot to align to the farther pole on the same side of the reef.
  */
  private static Pose2d findAutoAlignTarget() {
    Translation2d reefLocation;
    List<Pose2d> autoAlignLocations;

    // Determine which reef we're aligning to
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
      reefLocation = Constants.Field.REEF_LOCATION_RED;
      autoAlignLocations = Constants.Drive.AUTO_ALIGN_LOCATIONS_RED;
    } else {
      reefLocation = Constants.Field.REEF_LOCATION_BLUE;
      autoAlignLocations = Constants.Drive.AUTO_ALIGN_LOCATIONS_BLUE;
    }



    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/reefLocation",
        new Pose2d(reefLocation, new Rotation2d()));

    var drivetrain_state = s_drivetrain.getState();
    var drivetrain_pose = drivetrain_state.Pose;

    // find the angle to the reef
    Rotation2d angle =
        Rotation2d.fromRadians(
                Math.atan2(
                    drivetrain_pose.getY() - reefLocation.getY(),
                    drivetrain_pose.getX() - reefLocation.getX()));

    Pose2d left_pose;
    Pose2d right_pose;

    Translation2d left_offset = Constants.Drive.LEFT_BRANCH_OFFSET;
    Translation2d right_offset = Constants.Drive.RIGHT_BRANCH_OFFSET;
    
    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/left_offset_initial",
        left_offset);
    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/right_offset_initial",
        right_offset);
    
    double rotation_angle = 0.0;
    if (angle.getDegrees() <= 150.0 && angle.getDegrees() > 90.0) {
      rotation_angle = 300.0;
    }
    else if (angle.getDegrees() <= 90.0 && angle.getDegrees() > 30.0) {
      rotation_angle = 240.0;
    }
    else if (angle.getDegrees() <= 30.0 && angle.getDegrees() > -30.0) {
      rotation_angle = 180.0;
    }
    else if (angle.getDegrees() <= -30.0 && angle.getDegrees() > -90.0) {
      rotation_angle = 120.0;
    }
    else if (angle.getDegrees() <= -90.0 && angle.getDegrees() > -150.0) {
      rotation_angle = 60.0;
    }

    left_offset = left_offset.rotateBy(Rotation2d.fromDegrees(rotation_angle));
    right_offset = right_offset.rotateBy(Rotation2d.fromDegrees(rotation_angle));
    left_pose = new Pose2d(reefLocation.plus(left_offset), Rotation2d.fromDegrees(rotation_angle));
    right_pose = new Pose2d(reefLocation.plus(right_offset), Rotation2d.fromDegrees(rotation_angle));
    
    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/left_pose",
        left_pose);
    Logger.recordOutput(
        RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/right_pose",
        right_pose);

    return drivetrain_pose.nearest(Arrays.asList(left_pose, right_pose));
  }

  public void setDriveSpeed(double newSpeed) {
    s_driveSpeedScalar = newSpeed;
  }

  /**
   * Initialize hardware devices for drive subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware driveHardware = new Hardware();
    return driveHardware;
  }

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Set up stuff for limelight */
  public void limeLightSetup() {
    LimelightHelpers.SetRobotOrientation(
        "limelight1", s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation(
        "limelight1", s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
  }

  @Override
  public void periodic() {

    LoopTimer.addTimestamp(getName() + " Start");

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", true);
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      } else {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
      m_hasAppliedOperatorPerspective = true;
    } else {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", false);
    }

    double cameraTime = 0;
    double configTime = 0;
    double getPoseEstimateTime = 0;
    double rejectTagsTime = 0;
    double addMeasurementTime = 0;

    // String[] limelights = {"limelight-left", "limelight-right"};

    // for (String limelight : limelights) {
    //   cameraTime = Utils.getSystemTimeSeconds();
    //   LimelightHelpers.SetIMUMode(limelight, DriverStation.isDisabled() ? 1 : 2);
    //   LimelightHelpers.setLimelightNTDouble(limelight, "throttle_set", DriverStation.isDisabled()
    // ? 100 : 0);
    //   LimelightHelpers.SetRobotOrientation(limelight,
    // s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    //   configTime += Utils.getSystemTimeSeconds() - cameraTime;
    //   cameraTime = Utils.getSystemTimeSeconds();

    //   Logger.recordOutput(getName() + "/" + limelight + "/botpose",
    // LimelightHelpers.getBotPose3d_wpiBlue(limelight));
    //   double[] poseEntry = LimelightHelpers.getLimelightNTDoubleArray(limelight,
    // "botpose_orb_wpiblue");
    //   Logger.recordOutput(getName() + "/" + limelight + "/botpose_orb",
    // LimelightHelpers.toPose3D(poseEntry));
    //   LimelightHelpers.PoseEstimate pose_estimate =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

    //   getPoseEstimateTime += Utils.getSystemTimeSeconds() - cameraTime;
    //   cameraTime = Utils.getSystemTimeSeconds();

    //   if (pose_estimate == null) continue;
    //   boolean doRejectUpdate = false;
    //   if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
    //     int[] validIds = {6,7,8,9,10,11};
    //     LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
    //   }
    //   else {
    //     int[] validIds = {17,18,19,20,21,22};
    //     LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
    //   }
    //   if (s_drivetrain.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI) {
    //     doRejectUpdate = true;
    //   }

    //   if (pose_estimate.tagCount == 0) {
    //     doRejectUpdate = true;
    //   }

    //   rejectTagsTime += Utils.getSystemTimeSeconds() - cameraTime;
    //   cameraTime = Utils.getSystemTimeSeconds();

    //   if (!doRejectUpdate) {
    //     s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    //     s_drivetrain.addVisionMeasurement(pose_estimate.pose,
    // Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds));
    //     // Logger.recordOutput(getName() + "/" + limelight + "/botpose_orb", pose_estimate.pose);
    //   }

    //   addMeasurementTime += Utils.getSystemTimeSeconds() - cameraTime;

    //   LoopTimer.addTimestamp(limelight);
    // }

    Logger.recordOutput(getName() + "/cameraTimes/config", configTime);
    Logger.recordOutput(getName() + "/cameraTimes/getPoseEstimate", getPoseEstimateTime);
    Logger.recordOutput(getName() + "/cameraTimes/rejectTags", rejectTagsTime);
    Logger.recordOutput(getName() + "/cameraTimes/addMeasurement", addMeasurementTime);

    Logger.recordOutput(getName() + "/state", getState().toString());
    Logger.recordOutput(getName() + "/autoAlign/autotarget", findAutoAlignTarget());
    Logger.recordOutput(getName() + "/isNearSource", isNearSource());
    Logger.recordOutput(getName() + "/robotPose", s_drivetrain.getState().Pose);
    Logger.recordOutput(
        getName() + "/knownPose",
        new Pose2d(
            new Translation2d(
                Inches.of(144.0).minus(Feet.of(3)).minus(Inches.of((29.0 + (3.0 / 8.0)) / 2.0)),
                Inches.of(158.5)),
            new Rotation2d(Math.toRadians(0))));
    Logger.recordOutput(getName() + "/autoAlign/isAligned", s_isAligned);
    int i = 0;
    for (i = 0; i < 4; i++) {
      Logger.recordOutput(
          getName() + "/Mod" + i + "/torqueCurrent",
          s_drivetrain.getModule(i).getDriveMotor().getTorqueCurrent().getValue());
      Logger.recordOutput(
          getName() + "/Mod" + i + "/motorVoltage",
          s_drivetrain.getModule(i).getDriveMotor().getMotorVoltage().getValue());
    }
    LoopTimer.addTimestamp(getName() + " End");
  }

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }
}
