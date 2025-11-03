package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
// import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LoopTimer;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import gg.questnav.questnav.QuestNav;

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
                    Constants.Drive.MAX_ANGULAR_RATE
                        .times(-s_rotateRequest.getAsDouble())
                        .times(s_driveSpeedScalar)));
      }

      @Override
      public State nextState() {
        if (DriverStation.isAutonomous()) return AUTO;
        if (s_shouldAutoAlign
            && Math.abs(s_strafeRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR
            && Math.abs(s_driveRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR
            && Math.abs(s_rotateRequest.getAsDouble()) <= DriveSubsystem.DEADBAND_SCALAR) {
          requestAutoAlign();
          return AUTO_ALIGN;
        }
        return this;
      }
    },
    AUTO_ALIGN {
      long m_lastTime;
      long m_closeTime;

      Timer timer = new Timer();


      @Override
      public void initialize() {
        m_lastTime = System.currentTimeMillis();
        m_closeTime = System.currentTimeMillis();
        // move auto align away from the reef slightly
        // s_autoAlignTarget = s_autoAlignTarget.plus(new Transform2d(new Translation2d(-0.3, 0), new Rotation2d()));


        s_isAligned = false;
        timer.restart();
      }

      @Override
      public void execute() {
        double dt = (System.currentTimeMillis() - m_lastTime) / 1000.0;
        m_lastTime = System.currentTimeMillis();
        Logger.recordOutput("Drive/dt", dt);

        // Recompute the profile goal with the smallest error, thus giving the shortest
        // path. The goal
        // may be outside the input range after this operation, but that's OK because
        // the controller
        // will still go there and report an error of zero. In other words, the setpoint
        // only needs to
        // be offset from the measurement by the input range modulus; they don't need to
        // be equal

        Translation2d newPosition = s_autoAlignTarget.getTranslation().minus(s_drivetrain.getState().Pose.getTranslation());
        var drivetrain_state = s_drivetrain.getState();
        var drivetrain_pose = drivetrain_state.Pose;
        double distance =
            drivetrain_pose.getTranslation().getDistance(s_autoAlignTarget.getTranslation());

        var directionOfTravel = newPosition.getAngle();
        var outputVelocity = Math.min(
            Math.abs(s_autoDrive.calculate(distance, 0.0)), Constants.Drive.MAX_SPEED.magnitude()
          );

        var rotationRate = Math.min(
            Math.abs(s_headingController.calculate(s_drivetrain.getState().Pose.getRotation().getRadians())), s_autoAlignTarget.getRotation().getRadians()
          );

        var xComponent = outputVelocity * directionOfTravel.getCos();
        var yComponent = outputVelocity * directionOfTravel.getSin();

        s_drivetrain.setControl(
            s_driveRobotCentric
                .withVelocityX(MetersPerSecond.of(xComponent))
                .withVelocityY(MetersPerSecond.of(yComponent))
                .withRotationalRate(rotationRate));

        if(distance <= 0.05) {
          s_shouldAutoAlign = false;
        }
        
        Logger.recordOutput("DriveSubsystem/autoAlign/isClose", s_isClose);
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/closeTime", System.currentTimeMillis() - m_closeTime);

        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/x",
            s_drivetrain.getState().Pose.getX());
        Logger.recordOutput(
            "DriveSubsystem/autoAlign/error/y",
            s_drivetrain.getState().Pose.getY());
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
          if (!DriverStation.isAutonomous()) {
            return DRIVER_CONTROL;
          }
        }
        return this;
      }

      @Override
      public void end(boolean interrupted) {
        s_isAligned = false;

        s_drivetrain.setControl(s_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
    }
  }

  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  private static SwerveRequest.RobotCentric s_driveRobotCentric;
  private static PIDController s_autoDrive;
  private static PIDController s_headingController;
  private static QuestNav m_quest;
  private Transform2d ROBOT_TO_QUEST;
  private Transform2d OAKD_TO_ROBOT;
  private StructEntry<Pose3d> oakd_pose_entry;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static boolean s_shouldAutoAlign = false;
  private static Pose2d s_autoAlignTarget = new Pose2d();

  private static final Double DEADBAND_SCALAR = 0.085;

  /** Robot is within the auto align tolerance of the target point */
  private static boolean s_isAligned;

  /** Robot is within auto align tolerance * 2 of the target point */
  private static boolean s_isClose;

  // private static ArrayList<AprilTagCamera> m_cameras;

  private static double s_driveSpeedScalar = Constants.Drive.FAST_SPEED_SCALAR;

  //Camera variables
  private static boolean s_leftCameraSeesTag = false;
  private static boolean s_rightCameraSeesTag = false;


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
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDeadband(0)
            .withRotationalDeadband(0);

    s_autoDrive = new PIDController(3.6, 0, 0.);

    s_headingController = new PIDController(3.6, 0, 0);

    s_drivetrain.registerTelemetry(logger::telemeterize);



    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();

    m_quest = new QuestNav();
    ROBOT_TO_QUEST = new Transform2d(-0.1524, -0.3429, new Rotation2d((3 * Math.PI)/2));
    OAKD_TO_ROBOT = new Transform2d(Inches.of(-12.0), Inches.of(-17.5), Rotation2d.kZero);
    oakd_pose_entry = NetworkTableInstance.getDefault().getTable("PurpleRanger").getStructTopic("Pose", Pose3d.struct).getEntry(new Pose3d(), PubSubOption.keepDuplicates(true));
  }

  /**
   * Function to set up the LimeLights on the robot 
   */
  public void limelight_thread_func() {
    String[] limelights = {"limelight-left", "limelight-right"};

    while (true) {
      for (String limelight : limelights) {
        LimelightHelpers.SetIMUMode(limelight, DriverStation.isDisabled() ? 1 : 3);
        LimelightHelpers.setLimelightNTDouble(
            limelight, "throttle_set", DriverStation.isDisabled() ? 200 : 0);
        LimelightHelpers.SetRobotOrientation(
            limelight, s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Logger.recordOutput(
        //     getName() + "/" + limelight + "/botpose",
        //     LimelightHelpers.getBotPose3d_wpiBlue(limelight));
        double[] poseEntry =
            LimelightHelpers.getLimelightNTDoubleArray(limelight, "botpose_orb_wpiblue");
        // Logger.recordOutput(
        //     getName() + "/" + limelight + "/botpose_orb", LimelightHelpers.toPose3D(poseEntry));
        LimelightHelpers.PoseEstimate pose_estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

        if (pose_estimate == null) {
          if (limelight == "limelight-left") {
            s_leftCameraSeesTag = false;
          }
          if (limelight == "limelight-right") {
            s_rightCameraSeesTag = false;
          }
          continue;
        }
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
        // if (new Translation2d(s_drivetrain.getState().Speeds.vxMetersPerSecond, s_drivetrain.getState().Speeds.vyMetersPerSecond).getDistance(new Translation2d(0, 0)) > 2.0) {
        //   doRejectUpdate = true;
        // }

        if (pose_estimate.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.getRotation().getDegrees())) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          s_drivetrain.addVisionMeasurement(
              pose_estimate.pose, Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds));
          // Logger.recordOutput(getName() + "/" + limelight + "/botpose_orb", pose_estimate.pose);
        }
        if (limelight == "limelight-left") {
          s_leftCameraSeesTag = !doRejectUpdate;
        }
        if (limelight == "limelight-right") {
          s_rightCameraSeesTag = !doRejectUpdate;
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
    Logger.recordOutput(getName() + "/autoAlign/shouldAutoAlign", s_shouldAutoAlign);
  }

  /**
   * Gets the pose of the robot
   * @return The pose of the robot
   */
  public Pose2d getPose() {
    return s_drivetrain.getState().Pose;
  }

  /**
   * Checks if the robot is near the source
   * @return True if robot is near source
   */
  public boolean isNearSource() {
    return ((getPose().getX() < 2.0) && ((getPose().getY() < 1.5) || getPose().getY() > 6.0));
  }

  /**
   * Check whether the robot is aligned to the reef to score or not
   * @return A boolean showing whether the robot is aligned to the reef or not
   */
  public boolean isAligned() {
    return s_isAligned;
  }

  /**
   * Resets the pose of the robot to a 
   */
  public void resetPose() {
    s_drivetrain.resetPose();
  }

  /**
   * 
   * @param pose
   */
  public void resetPose(Pose2d pose) {
    s_drivetrain.resetPose(pose);
  }

  /**
   * Finds the nearest reef pole to align to from the robot's pose
   * @return A pose closest aligned to the nearest reef pole from the robot's current pose
   */
  private static Pose2d findAutoAlignTarget() {
    return findAutoAlignTarget(s_drivetrain.getState().Pose);
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

  /**
   * Boolean if either of the limelights sees an april tag
   * @return boolean if either of the limelights sees an AprilTag
   */
	public boolean seesTag() {
		return (s_leftCameraSeesTag || s_rightCameraSeesTag);
	}

  /**
   * Initialize hardware devices for drive subsystem
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

  public void questNavReset() {
    Pose2d robotPose = getPose();
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    m_quest.setPose(questPose);
  }

  public void getQuestNavPose() {
    if (m_quest.isConnected() && m_quest.isTracking()) {
      Pose2d questPose = m_quest.getPose();
      Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
      Logger.recordOutput(getName() + "Drive/actualQuestRobotPose", robotPose);
    }
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

    m_quest.commandPeriodic(); 


    Logger.recordOutput(getName() + "/cameraTimes/config", configTime);
    Logger.recordOutput(getName() + "/cameraTimes/getPoseEstimate", getPoseEstimateTime);
    Logger.recordOutput(getName() + "/cameraTimes/rejectTags", rejectTagsTime);
    Logger.recordOutput(getName() + "/cameraTimes/addMeasurement", addMeasurementTime);

    Logger.recordOutput(getName() + "/state", getState().toString());
    Logger.recordOutput(getName() + "/autoAlign/autotarget", findAutoAlignTarget());
    Logger.recordOutput(getName() + "/isNearSource", isNearSource());
    Logger.recordOutput(getName() + "/robotPose", s_drivetrain.getState().Pose);
    Logger.recordOutput(getName() + "/seesTag", seesTag());

    Logger.recordOutput(getName() + "/PurpleRangerPose", oakd_pose_entry.get().toPose2d().transformBy(OAKD_TO_ROBOT));

    getQuestNavPose();

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