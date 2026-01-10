package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LimelightHelpers;
import gg.questnav.questnav.QuestNav;

public class DriveHardware {
  private static final class DummySubsystem implements Subsystem {};
  private static final DummySubsystem s_dummyInstance = new DummySubsystem();

  public static TunerConstants.TunerSwerveDrivetrain s_drivetrain;
  private static final SwerveRequest.ApplyRobotSpeeds s_pathApplyRobotSpeeds =
    new SwerveRequest.ApplyRobotSpeeds();
  /* Keep track if we've ever applied the operator perspective before or not */
  private static boolean s_hasAppliedOperatorPerspective = false;

  // Limelight
  private static boolean s_leftCameraSeesTag = false;
  private static boolean s_rightCameraSeesTag = false;
  private static Thread s_limelight_thread;

  // Quest
  private static QuestNav s_quest;
  private static final Transform2d ROBOT_TO_QUEST = new Transform2d(
    -0.1524,
    -0.3429,
    new Rotation2d((3 * Math.PI)/2)
  );

  // Oak-D
  private static final Transform2d OAKD_TO_ROBOT = new Transform2d(
    Inches.of(-12.0),
    Inches.of(-17.5),
    Rotation2d.kZero
  );
  private static StructEntry<Pose3d> s_oakd_pose_entry;

  public static void initialize() {
    s_drivetrain = TunerConstants.createDrivetrain();
    DriveHardware.s_drivetrain.registerTelemetry(
      (state) -> {
        var name = DriveHardware.class.getSimpleName();
        // Telemeterize the swerve drive state
        Logger.recordOutput(name + "/CTRESwerveState/Pose", state.Pose);
        Logger.recordOutput(name + "/CTRESwerveState/RawHeading", state.RawHeading);
        Logger.recordOutput(name + "/CTRESwerveState/Speeds", state.Speeds);
        Logger.recordOutput(name + "/CTRESwerveState/ModuleStates", state.ModuleStates);
        Logger.recordOutput(name + "/CTRESwerveState/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput(name + "/CTRESwerveState/ModulePositions", state.ModulePositions);
        Logger.recordOutput(name + "/CTRESwerveState/Timestamp", state.Timestamp);
        Logger.recordOutput(name + "/CTRESwerveState/OdometryPeriod", state.OdometryPeriod);
      }
    );

    s_limelight_thread = new Thread(DriveHardware::limelight_thread_func);
    s_limelight_thread.setDaemon(true);
    s_limelight_thread.start();

    s_quest = new QuestNav();

    s_oakd_pose_entry = NetworkTableInstance.getDefault()
      .getTable("PurpleRanger")
      .getStructTopic("Pose", Pose3d.struct)
      .getEntry(new Pose3d(), PubSubOption.keepDuplicates(true));
    
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        () -> DriveHardware.s_drivetrain.getState().Pose,
        DriveHardware::resetPoseNotGyro,
        () -> s_drivetrain.getState().Speeds,
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) -> {
          s_drivetrain.setControl(
            s_pathApplyRobotSpeeds
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
              .withDriveRequestType(DriveRequestType.Velocity)
              .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          );
        },
        new PPHolonomicDriveController(
          // translation
          new PIDConstants(2, 0, 0),
          // rotation
          new PIDConstants(1, 0, 0)
        ),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
        s_dummyInstance
      );
    } catch (Exception ex) {
      DriverStation.reportError(
        "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public static void resetPoseNotGyro(Pose2d pose) {
    s_drivetrain.resetPose(
      new Pose2d(
        pose.getX(),
        pose.getY(),
        s_drivetrain.getState().Pose.getRotation()
      )
    );
  }

  public static void resetRotation() {
    var current_pose = s_drivetrain.getState().Pose;
    var new_rotation = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) ?
      Rotation2d.k180deg :
      Rotation2d.kZero;
    s_drivetrain.resetPose(
      new Pose2d(
        current_pose.getX(),
        current_pose.getY(),
        new_rotation)
    );
  }

  /**
   * Boolean if either of the limelights sees an april tag
   * @return boolean if either of the limelights sees an AprilTag
   */
	public static boolean seesTag() {
		return (s_leftCameraSeesTag || s_rightCameraSeesTag);
	}

  public static void questNavReset() {
    Pose2d robotPose = s_drivetrain.getState().Pose;
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    s_quest.setPose(questPose);
  }

  private static Pose2d getQuestNavPose() {
    if (!s_quest.isConnected() || !s_quest.isTracking()) {
      return null;
    }
    Pose2d questPose = s_quest.getPose();
    return questPose.transformBy(ROBOT_TO_QUEST.inverse());
  }

  private static void limelight_thread_func() {
    String[] limelights = {"limelight-left", "limelight-right"};

    while (true) {
      for (String limelight : limelights) {
        LimelightHelpers.SetIMUMode(
          limelight,
          DriverStation.isDisabled() ? 1 : 3
        );
        LimelightHelpers.setLimelightNTDouble(
          limelight, "throttle_set",
          DriverStation.isDisabled() ? 200 : 0
        );
        LimelightHelpers.SetRobotOrientation(
          limelight,
          DriveHardware.s_drivetrain.getState().Pose.getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0
        );
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
          int[] validIds = {6, 7, 8, 9, 10, 11};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        } else {
          int[] validIds = {17, 18, 19, 20, 21, 22};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        }

        var pose_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
          limelight
        );

        if (
          DriveHardware.s_drivetrain.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI ||
          pose_estimate == null ||
          pose_estimate.tagCount == 0 ||
          Double.isNaN(pose_estimate.pose.getX()) ||
          Double.isNaN(pose_estimate.pose.getY()) ||
          Double.isNaN(pose_estimate.pose.getRotation().getDegrees())
        ) {
          if (limelight == "limelight-left") {
            s_leftCameraSeesTag = false;
          }
          if (limelight == "limelight-right") {
            s_rightCameraSeesTag = false;
          }
          continue;
        }

        DriveHardware.s_drivetrain.setVisionMeasurementStdDevs(
          VecBuilder.fill(0.7, 0.7, 9999999)
        );
        DriveHardware.s_drivetrain.addVisionMeasurement(
          pose_estimate.pose,
          Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds)
        );
        if (limelight == "limelight-left") {
          s_leftCameraSeesTag = true;
        }
        if (limelight == "limelight-right") {
          s_rightCameraSeesTag = true;
				}
      }
      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {}
    }
  }

  public static void periodic() {
    var name = DriveHardware.class.getSimpleName();

    s_quest.commandPeriodic();

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should
     * apply it regardless of DS state. This allows us to correct the
     * perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is 
     * disabled. This ensures driving behavior doesn't change until an explicit
     * disable event occurs during testing.
     */
    if (!s_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriveHardware.s_drivetrain.setOperatorPerspectiveForward(
        DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) ?
          Rotation2d.k180deg :
          Rotation2d.kZero
      );
      s_hasAppliedOperatorPerspective = true;
    }
    Logger.recordOutput(name + "/robotPose", DriveHardware.s_drivetrain.getState().Pose);
    Logger.recordOutput(name + "/seesTag", seesTag());
    Logger.recordOutput(name + "/PurpleRangerPose", s_oakd_pose_entry.get().toPose2d().transformBy(OAKD_TO_ROBOT));
    Logger.recordOutput(name + "/actualQuestRobotPose", getQuestNavPose());

    Logger.recordOutput(
      name + "/knownPose",
      new Pose2d(
        new Translation2d(
          Inches.of(144.0)
            .minus(Feet.of(3))
            .minus(Inches.of((29.0 + (3.0 / 8.0)) / 2.0)),
          Inches.of(158.5)
        ),
        new Rotation2d(Math.toRadians(0))
      )
    );
    for (int i = 0; i < 4; i++) {
      var module = DriveHardware.s_drivetrain.getModule(i);
      Logger.recordOutput(
          name + "/Mod" + i + "/torqueCurrent",
          module.getDriveMotor().getTorqueCurrent().getValue());
      Logger.recordOutput(
          name + "/Mod" + i + "/motorVoltage",
          module.getDriveMotor().getMotorVoltage().getValue());
    }
  }
}
