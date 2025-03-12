package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware() {
  }

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
        if (s_shouldAutoAlign)
          return AUTO_ALIGN;
        if (!DriverStation.isAutonomous())
          return DRIVER_CONTROL;
        return this;
      }

    },
    DRIVER_CONTROL {
      @Override
      public void execute() {
        s_drivetrain.setControl(s_drive
            .withVelocityX(Constants.Drive.MAX_SPEED.times(-Math.pow(s_strafeRequest.getAsDouble(), 1)))
            .withVelocityY(Constants.Drive.MAX_SPEED.times(-Math.pow(s_driveRequest.getAsDouble(), 1)))
            .withRotationalRate(Constants.Drive.MAX_ANGULAR_RATE.times(-s_rotateRequest.getAsDouble())));
      }

      @Override
      public State nextState() {
        if (s_shouldAutoAlign)
          return AUTO_ALIGN;
        if (DriverStation.isAutonomous())
          return AUTO;
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
        System.out.println(s_drivetrain.getState().Pose.getRotation().getRadians() + "\n");
        m_currentTurnState = new TrapezoidProfile.State(
            s_drivetrain.getState().Pose.getRotation().getRadians(),
            0);

        m_currentDriveXState = new TrapezoidProfile.State(
            s_drivetrain.getState().Pose.getX(),
            0);

        m_currentDriveYState = new TrapezoidProfile.State(
            s_drivetrain.getState().Pose.getY(),
            0);

        s_isAligned = false;
      }

      @Override
      public void execute() {
        double dt = (System.currentTimeMillis() - m_lastTime) / 1000.0;
        m_lastTime = System.currentTimeMillis();
        Logger.recordOutput("Drive/dt", dt);

        /*
         * // if we're too far from the drivetrain target...
         * if (Math.abs(m_currentDriveXState.position -
         * s_drivetrain.getState().Pose.getX()) > 0.25
         * || Math.abs(m_currentDriveYState.position -
         * s_drivetrain.getState().Pose.getY()) > 0.25
         * || Math.abs(m_currentTurnState.position -
         * s_drivetrain.getState().Pose.getRotation().getRadians()) > 0.1
         * ) {
         * // move the drivetrain target so the profile can correct for the error
         * m_currentDriveXState = new
         * TrapezoidProfile.State(s_drivetrain.getState().Pose.getX(),
         * s_drivetrain.getState().Speeds.vxMetersPerSecond);
         * m_currentDriveYState = new
         * TrapezoidProfile.State(s_drivetrain.getState().Pose.getY(),
         * s_drivetrain.getState().Speeds.vyMetersPerSecond);
         * m_currentTurnState = new
         * TrapezoidProfile.State(s_drivetrain.getState().Pose.getRotation().getRadians(
         * ), s_drivetrain.getState().Speeds.omegaRadiansPerSecond);
         * }
         */

        // Get error which is the smallest distance between goal and measurement
        double errorBound = Math.PI;
        double measurement = s_drivetrain.getState().Pose.getRotation().getRadians();
        double goalMinDistance = MathUtil.inputModulus(s_autoAlignTargetTurn.position - measurement,
            -errorBound, errorBound);
        double setpointMinDistance = MathUtil.inputModulus(m_currentTurnState.position - measurement,
            -errorBound, errorBound);

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

        m_currentDriveXState = s_driveProfile.calculate(dt, m_currentDriveXState, s_autoAlignTargetDriveX);
        m_currentDriveYState = s_driveProfile.calculate(dt, m_currentDriveYState, s_autoAlignTargetDriveY);
        m_currentTurnState = s_turnProfile.calculate(dt, m_currentTurnState, s_autoAlignTargetTurn);

        double distance = s_drivetrain.getState().Pose.getTranslation().getDistance(s_autoAlignTarget.getTranslation());
        double heading = Math
            .abs((s_drivetrain.getState().Pose.getRotation().getRadians() - s_autoAlignTargetTurn.position)) % 360;

        // if the robot is _very_ close to the target, turn off the drivetrain
        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE / 2
            && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN / 2
                || heading > (Math.PI * 2 - (Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN / 2)))) {
          s_drivetrain.setControl(s_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
          Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", true);
        } else {
          s_drivetrain.setControl(
            s_autoDrive
                .withTargetDirection(new Rotation2d(m_currentTurnState.position))
                .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
                .withTargetX(m_currentDriveXState.position)
                .withFeedforwardX(m_currentDriveXState.velocity)
                .withTargetY(m_currentDriveYState.position)
                .withFeedforwardY(m_currentDriveYState.velocity));
          // if (DriverStation.getAlliance().isEmpty()) {
          //   Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/alliance", "none");
          // } else {
          //   if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
          //     s_drivetrain.setControl(
          //         s_autoDrive
          //             .withTargetDirection(new Rotation2d(m_currentTurnState.position))
          //             .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
          //             .withTargetX(m_currentDriveXState.position)
          //             .withFeedforwardX(m_currentDriveXState.velocity)
          //             .withTargetY(m_currentDriveYState.position)
          //             .withFeedforwardY(m_currentDriveYState.velocity));
          //     Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/alliance", "blue");
          //   } else if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
          //     s_drivetrain.setControl(
          //         s_autoDrive
          //             .withTargetDirection(new Rotation2d(m_currentTurnState.position))
          //             .withTargetRateFeedforward(Units.RadiansPerSecond.of(m_currentTurnState.velocity))
          //             .withTargetX(-m_currentDriveXState.position)
          //             .withFeedforwardX(-m_currentDriveXState.velocity)
          //             .withTargetY(m_currentDriveYState.position)
          //             .withFeedforwardY(m_currentDriveYState.velocity));
          //     Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/alliance", "red");
          //sigma boy
          //   }
          // }
          Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/isVeryAligned", false);
        }

        Logger.recordOutput("DriveSubsystem/autoAlign/targetPose", new Pose2d(m_currentDriveXState.position,
            m_currentDriveYState.position, new Rotation2d(m_currentTurnState.position)));
        Logger.recordOutput("DriveSubsystem/autoAlign/finalPose", new Pose2d(s_autoAlignTargetDriveX.position,
            s_autoAlignTargetDriveY.position, new Rotation2d(s_autoAlignTargetTurn.position)));

        Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/distanceError", distance);
        Logger.recordOutput(RobotContainer.DRIVE_SUBSYSTEM.getName() + "/autoAlign/headingError", heading);
        if (distance < Constants.Drive.AUTO_ALIGN_TOLERANCE && (heading < Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN
            || heading > (Math.PI * 2 - Constants.Drive.AUTO_ALIGN_TOLERANCE_TURN))) {
          s_isAligned = true;
        }
      }

      @Override
      public State nextState() {
        if (!s_shouldAutoAlign) {
          if (DriverStation.isAutonomous())
            return AUTO;
          else
            return DRIVER_CONTROL;
        }
        if (s_isAligned && DriverStation.isTeleop()) {
          return DRIVER_CONTROL;
        }
        return this;
      }

      @Override
      public void end(boolean interrupted) {
        s_shouldAutoAlign = false;
        s_isAligned = false;
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
  private static Pose2d s_autoAlignTarget = new Pose2d();
  private static TrapezoidProfile.State s_autoAlignTargetDriveX;
  private static TrapezoidProfile.State s_autoAlignTargetDriveY;
  private static TrapezoidProfile.State s_autoAlignTargetTurn;

  private static TrapezoidProfile s_turnProfile;
  private static TrapezoidProfile s_driveProfile;

  private static boolean s_isAligned;

  private static ArrayList<AprilTagCamera> m_cameras;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> s_drivetrain.setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> s_drivetrain.setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            s_drivetrain.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  public SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

  public DriveSubsystem(Hardware driveHardware, Telemetry logger) {
    super(State.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();
    /* Setting up bindings for necessary control of the swerve drive platform */
    s_drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(0.05))
        .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    s_autoDrive = new FieldCentricWithPose()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    s_autoDrive.HeadingController.setPID(5, 0, 0);
    s_autoDrive.HeadingController.enableContinuousInput(0, Math.PI * 2);

    s_autoDrive.XController.setPID(2, 0, 0);
    s_autoDrive.YController.setPID(2, 0, 0);

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
   * Request that the drivetrain aligns to the reef
   * 
   * @param state
   */
  public void requestAutoAlign(Pose2d pose) {
    s_autoAlignTarget = pose;
    s_autoAlignTargetDriveX = new TrapezoidProfile.State(pose.getX(), 0);
    s_autoAlignTargetDriveY = new TrapezoidProfile.State(pose.getY(), 0);
    s_autoAlignTargetTurn = new TrapezoidProfile.State(pose.getRotation().getRadians(), 0);
    s_shouldAutoAlign = true;
  }

  public void requestAutoAlign() {
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

  /**
   * Returns the location the robot should go to in order to align to the nearest
   * reef pole
   */
  private Pose2d findAutoAlignTarget() {
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

    Logger.recordOutput(getName() + "/autoAlign/reefLocation", new Pose2d(reefLocation, new Rotation2d()));

    // find the angle to the reef
    double angle = Math
        .toDegrees(Math.atan2(s_drivetrain.getState().Pose.getX() - reefLocation.getX(),
            s_drivetrain.getState().Pose.getY() - reefLocation.getY()))
        + 360;
    // Convert to an index (0-11)
    int index = (int) ((((angle / 30)) + 10) % 12);

    // Calculate the final position
    Translation2d position;
    // initial offset
    if (index % 2 == 1)
      position = Constants.Drive.LEFT_BRANCH_OFFSET;
    else
      position = Constants.Drive.RIGHT_BRANCH_OFFSET;
    position = position.rotateBy(new Rotation2d(Degrees.of((int) (index / 2) * -60))); // move to correct side of reef
    position = position.plus(reefLocation); // move to actual reef location
    // position = position.plus(autoAlignLocations.get(index).getTranslation()); //
    // apply custom offset for this position

    Logger.recordOutput(getName() + "/autoAlign/targetBranch", index % 2 == 0);

    return new Pose2d(position, autoAlignLocations.get(index).getRotation());
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

  /**
   * Set up stuff for limelight
   */
  public void limeLightSetup() {
    LimelightHelpers.SetRobotOrientation("limelight1", s_drivetrain.getState().Pose.getRotation().getDegrees(), 
    0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation("limelight1", s_drivetrain.getState().Pose.getRotation().getDegrees(), 
    0, 0, 0, 0, 0);
  }

  @Override
  public void periodic() {
    
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
        s_drivetrain.setOperatorPerspectiveForward(CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      } else {
        s_drivetrain.setOperatorPerspectiveForward(CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
      m_hasAppliedOperatorPerspective = true;
    }
    else {
        Logger.recordOutput(getName() + "/settingOperatorPerspective", false);
    }

    String[] limelights = {"limelight1", "limelight2"};

    for (String limelight : limelights) {
      LimelightHelpers.SetIMUMode(limelight, DriverStation.isDisabled() ? 1 : 2);
      LimelightHelpers.setLimelightNTDouble(limelight, "throttle_set", DriverStation.isDisabled() ? 100 : 0);
      LimelightHelpers.SetRobotOrientation(limelight, s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate pose_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
      boolean doRejectUpdate = false;
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) { 
        int[] validIds = {6,7,8,9,10,11};
        LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
      } 
      else {
        int[] validIds = {17,18,19,20,21,22};
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
        s_drivetrain.addVisionMeasurement(pose_estimate.pose, pose_estimate.timestampSeconds);
      }
    }

    Logger.recordOutput(getName() + "/state", getState().toString());
    Logger.recordOutput(getName() + "/autoAlign/autotarget", findAutoAlignTarget());
    Logger.recordOutput(getName() + "/isNearSource", isNearSource());
    Logger.recordOutput(getName() + "/robotPose", s_drivetrain.getState().Pose);
    Logger.recordOutput("temp", new Pose2d(new Translation2d(3.175, 4.0159), new Rotation2d(Math.toRadians(0))));
    Logger.recordOutput(getName() + "/autoAlign/isAligned", s_isAligned);
    int i = 0;
    for (i = 0; i < 4; i++) {
      Logger.recordOutput(getName() + "/Mod" + i + "/torqueCurrent",
          s_drivetrain.getModule(i).getDriveMotor().getTorqueCurrent().getValue());
      Logger.recordOutput(getName() + "/Mod" + i + "/motorVoltage",
          s_drivetrain.getModule(i).getDriveMotor().getMotorVoltage().getValue());
    }
  }

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }
}
