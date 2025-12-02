package frc.robot.subsystems.localization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;

public class localizationSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-left";
    private final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    private final NetworkTable logTable = NetworkTableInstance.getDefault().getTable("LocalizationSubsystem");

    private static final double TARGET_DISTANCE_FEET = 2.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    private static final double DISTANCE_TOLERANCE_FEET = 0.3;
    
    private static final double CAMERA_MOUNT_ANGLE_X_DEGREES = 40.0;
    private static final double CAMERA_MOUNT_ANGLE_Y_DEGREES = 99.846552;
    private static final double CAMERA_HEIGHT_INCHES = 9.5;
    private static final double CAMERA_OFFSET_X_INCHES = 8.41;
    private static final double CAMERA_OFFSET_Y_INCHES = 11.6;

    private AprilTagFieldLayout aprilTagFieldLayout;
    
    private double lastUpdate = 0.0;

    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double[] tcornxy = new double[8];
    private int tagId = -1;

    public localizationSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        } catch (Exception e) {
            Logger.recordOutput("Localization/AprilTagLayoutError", e.getMessage());
        }
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastUpdate < 0.011)
            return;
        lastUpdate = now;

        updateFromLimelight();

        if (isCornersValid() && tagId != -1) {
            calculate3DPosition();
            calculateTagArea();
            calculateDiagonalsAndMidpoint();
            calculateOptimalMovement();
            calculateAndLogPoseEstimation();
        } else {
            Logger.recordOutput("Localization/Status", "No valid corners");
            Logger.recordOutput("Movement/Command", "STOP - No target detected");
        }
    }

    private void updateFromLimelight() {
        double tv = llTable.getEntry("tv").getDouble(0);
        tx = llTable.getEntry("tx").getDouble(0);
        ty = llTable.getEntry("ty").getDouble(0);
        ta = llTable.getEntry("ta").getDouble(0);
        tcornxy = llTable.getEntry("tcornxy").getDoubleArray(new double[8]);
        tagId = (int) llTable.getEntry("tid").getDouble(-1);

        if (tv == 1) {
            logTable.getEntry("tx").setDouble(tx);
            logTable.getEntry("ty").setDouble(ty);
            logTable.getEntry("ta").setDouble(ta);
            logTable.getEntry("tcornxy").setDoubleArray(tcornxy);
            logTable.getEntry("tid").setDouble(tagId);
            logTable.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        }

        Logger.recordOutput("Localization/tv", tv);
        Logger.recordOutput("Localization/tx", tx);
        Logger.recordOutput("Localization/ty", ty);
        Logger.recordOutput("Localization/ta", ta);
        Logger.recordOutput("Localization/tagId", tagId);
    }

    private boolean isCornersValid() {
        if (tcornxy == null || tcornxy.length < 8)
            return false;
        for (double v : tcornxy) {
            if (Math.abs(v) > 1e-3)
                return true;
        }
        return false;
    }

    private void calculate3DPosition() {
        double y0 = tcornxy[1], y1 = tcornxy[3], y2 = tcornxy[5], y3 = tcornxy[7];
        double x0 = tcornxy[0], x1 = tcornxy[2], x2 = tcornxy[4], x3 = tcornxy[6];

        double hLeft = Math.abs(y3 - y0);
        double hRight = Math.abs(y2 - y1);
        double wTop = Math.abs(x2 - x3);
        double wBottom = Math.abs(x1 - x0);

        Logger.recordOutput("Localization/hLeft", hLeft);
        Logger.recordOutput("Localization/hRight", hRight);
        Logger.recordOutput("Localization/wTop", wTop);
        Logger.recordOutput("Localization/wBottom", wBottom);
    }

    private double calculateTagArea() {
        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        double hLeft = Math.abs(y3 - y0);
        double hRight = Math.abs(y2 - y1);
        double wTop = Math.abs(x2 - x3);
        double wBottom = Math.abs(x1 - x0);

        double avgHeight = (hLeft + hRight) / 2.0;
        double avgWidth = (wTop + wBottom) / 2.0;
        double area = avgWidth * avgHeight;

        Logger.recordOutput("Localization/avgHeight", avgHeight);
        Logger.recordOutput("Localization/avgWidth", avgWidth);
        Logger.recordOutput("Localization/TagArea", area);

        return area;
    }

    private void calculateDiagonalsAndMidpoint() {
        double x0 = tcornxy[0], y0 = tcornxy[1];
        double x1 = tcornxy[2], y1 = tcornxy[3];
        double x2 = tcornxy[4], y2 = tcornxy[5];
        double x3 = tcornxy[6], y3 = tcornxy[7];

        double diag1 = Math.sqrt(Math.pow(x2 - x0, 2) + Math.pow(y2 - y0, 2));
        double diag2 = Math.sqrt(Math.pow(x3 - x1, 2) + Math.pow(y3 - y1, 2));
        double midX = (x0 + x1 + x2 + x3) / 4.0;
        double midY = (y0 + y1 + y2 + y3) / 4.0;

        Logger.recordOutput("Localization/Diagonal1", diag1);
        Logger.recordOutput("Localization/Diagonal2", diag2);
        Logger.recordOutput("Localization/MidpointX", midX);
        Logger.recordOutput("Localization/MidpointY", midY);
    }

    private void calculateOptimalMovement() {
        double realTimeTA = calculateTagArea();
        double cameraDistanceToTag = calculateDistanceFromArea(realTimeTA);
        
        double cameraOffsetDistance = Math.sqrt(
            Math.pow(CAMERA_OFFSET_X_INCHES / 12.0, 2) + 
            Math.pow(CAMERA_OFFSET_Y_INCHES / 12.0, 2)
        );
        
        double horizontalAngleError = tx;
        
        double adjustedCameraDistance = cameraDistanceToTag / Math.cos(Math.toRadians(CAMERA_MOUNT_ANGLE_Y_DEGREES));
        
        double robotCenterDistanceToTag = Math.sqrt(
            Math.pow(adjustedCameraDistance, 2) + Math.pow(cameraOffsetDistance, 2) - 
            2 * adjustedCameraDistance * cameraOffsetDistance * 
            Math.cos(Math.toRadians(horizontalAngleError + CAMERA_MOUNT_ANGLE_X_DEGREES))
        );
        
        double currentDistance = robotCenterDistanceToTag + 1.0;
        
        Logger.recordOutput("Movement/CameraDistance_Feet", cameraDistanceToTag);
        Logger.recordOutput("Movement/RobotCenterDistance_Feet", robotCenterDistanceToTag);
        Logger.recordOutput("Movement/CurrentDistance_Feet", currentDistance);
        Logger.recordOutput("Movement/TargetDistance_Feet", TARGET_DISTANCE_FEET);
        Logger.recordOutput("Movement/CameraOffset_Feet", cameraOffsetDistance);
        
        Logger.recordOutput("Movement/HorizontalAngleError_Degrees", horizontalAngleError);
        
        double forwardMovement = currentDistance - TARGET_DISTANCE_FEET;
        Logger.recordOutput("Movement/RequiredForward_Feet", forwardMovement);
        
        double strafeMovement = currentDistance * Math.tan(Math.toRadians(horizontalAngleError));
        Logger.recordOutput("Movement/RequiredStrafe_Feet", strafeMovement);
        
        double rotationRequired = horizontalAngleError;
        Logger.recordOutput("Movement/RequiredRotation_Degrees", rotationRequired);
        
        String movementCommand = generateMovementCommand(forwardMovement, strafeMovement, rotationRequired);
        Logger.recordOutput("Movement/Command", movementCommand);
        
        boolean isAligned = checkAlignment(forwardMovement, rotationRequired);
        Logger.recordOutput("Movement/IsAligned", isAligned);
        
        logMovementBreakdown(forwardMovement, strafeMovement, rotationRequired);
    }

    private void calculateAndLogPoseEstimation() {
        if (aprilTagFieldLayout == null) {
            Logger.recordOutput("PoseEstimation/Status", "AprilTag layout not loaded");
            return;
        }

        if (tagId == -1) {
            Logger.recordOutput("PoseEstimation/Status", "No valid tag detected");
            return;
        }

        Optional<edu.wpi.first.math.geometry.Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
        if (!tagPoseOptional.isPresent()) {
            Logger.recordOutput("PoseEstimation/Status", "Tag ID " + tagId + " not in field layout");
            Logger.recordOutput("PoseEstimation/TagID", tagId);
            return;
        }

        edu.wpi.first.math.geometry.Pose3d tagPose3d = tagPoseOptional.get();
        Pose2d tagPose = tagPose3d.toPose2d();
        
        double realTimeTA = calculateTagArea();
        double cameraDistanceToTag = calculateDistanceFromArea(realTimeTA);
        
        double cameraOffsetDistance = Math.sqrt(
            Math.pow(CAMERA_OFFSET_X_INCHES / 12.0, 2) + 
            Math.pow(CAMERA_OFFSET_Y_INCHES / 12.0, 2)
        );
        
        double horizontalAngleError = tx;
        
        double adjustedCameraDistance = cameraDistanceToTag / Math.cos(Math.toRadians(CAMERA_MOUNT_ANGLE_Y_DEGREES));
        
        double robotCenterDistanceToTag = Math.sqrt(
            Math.pow(adjustedCameraDistance, 2) + Math.pow(cameraOffsetDistance, 2) - 
            2 * adjustedCameraDistance * cameraOffsetDistance * 
            Math.cos(Math.toRadians(horizontalAngleError + CAMERA_MOUNT_ANGLE_X_DEGREES))
        );
        
        double currentDistance = robotCenterDistanceToTag + 1.0;
        double distanceMeters = currentDistance * 0.3048;
        
        double robotAngleToTag = tagPose.getRotation().getDegrees() + 180 - horizontalAngleError;
        Rotation2d robotRotation = Rotation2d.fromDegrees(robotAngleToTag);
        
        double angleRad = Math.toRadians(robotAngleToTag);
        double robotX = tagPose.getX() - distanceMeters * Math.cos(angleRad);
        double robotY = tagPose.getY() - distanceMeters * Math.sin(angleRad);
        
        Translation2d robotTranslation = new Translation2d(robotX, robotY);
        Pose2d estimatedPose = new Pose2d(robotTranslation, robotRotation);
        
        double timestamp = Timer.getFPGATimestamp();
        
        Logger.recordOutput("PoseEstimation/TagID", tagId);
        Logger.recordOutput("PoseEstimation/TagPoseX", tagPose.getX());
        Logger.recordOutput("PoseEstimation/TagPoseY", tagPose.getY());
        Logger.recordOutput("PoseEstimation/TagRotation", tagPose.getRotation().getDegrees());
        Logger.recordOutput("PoseEstimation/EstimatedPoseX", estimatedPose.getX());
        Logger.recordOutput("PoseEstimation/EstimatedPoseY", estimatedPose.getY());
        Logger.recordOutput("PoseEstimation/EstimatedRotation", estimatedPose.getRotation().getDegrees());
        Logger.recordOutput("PoseEstimation/DistanceToTag_Meters", distanceMeters);
        Logger.recordOutput("PoseEstimation/Timestamp", timestamp);
        Logger.recordOutput("PoseEstimation/EstimatedPose2d", estimatedPose);
        Logger.recordOutput("PoseEstimation/Status", "Valid");
    }

    private String generateMovementCommand(double forward, double strafe, double rotation) {
        StringBuilder command = new StringBuilder();
        
        if (Math.abs(forward) < DISTANCE_TOLERANCE_FEET && 
            Math.abs(rotation) < ANGLE_TOLERANCE_DEGREES) {
            return "ALIGNED - Hold position";
        }
        
        if (Math.abs(rotation) > ANGLE_TOLERANCE_DEGREES) {
            command.append(String.format("ROTATE %.1f° %s", 
                Math.abs(rotation), 
                rotation > 0 ? "RIGHT" : "LEFT"));
            command.append(" → THEN → ");
        }
        
        if (Math.abs(forward) > DISTANCE_TOLERANCE_FEET) {
            command.append(String.format("DRIVE %.2f ft %s", 
                Math.abs(forward), 
                forward > 0 ? "BACKWARD" : "FORWARD"));
        } else {
            command.append("HOLD DISTANCE");
        }
        
        if (Math.abs(strafe) > 0.1) {
            command.append(String.format(" + STRAFE %.2f ft %s", 
                Math.abs(strafe), 
                strafe > 0 ? "RIGHT" : "LEFT"));
        }
        
        return command.toString();
    }

    private boolean checkAlignment(double forwardError, double rotationError) {
        return Math.abs(forwardError) < DISTANCE_TOLERANCE_FEET && 
               Math.abs(rotationError) < ANGLE_TOLERANCE_DEGREES;
    }

    private void logMovementBreakdown(double forward, double strafe, double rotation) {
        Logger.recordOutput("Movement/Phase1_RotationNeeded", Math.abs(rotation) > ANGLE_TOLERANCE_DEGREES);
        Logger.recordOutput("Movement/Phase2_ForwardNeeded", Math.abs(forward) > DISTANCE_TOLERANCE_FEET);
        Logger.recordOutput("Movement/Phase3_StrafeNeeded", Math.abs(strafe) > 0.1);
        
        Logger.recordOutput("Movement/Direction_Forward", forward < 0);
        Logger.recordOutput("Movement/Direction_Backward", forward > 0);
        Logger.recordOutput("Movement/Direction_StrafeLeft", strafe < 0);
        Logger.recordOutput("Movement/Direction_StrafeRight", strafe > 0);
        Logger.recordOutput("Movement/Direction_RotateLeft", rotation < 0);
        Logger.recordOutput("Movement/Direction_RotateRight", rotation > 0);
        
        Logger.recordOutput("Movement/ForwardError_Inches", forward * 12);
        Logger.recordOutput("Movement/StrafeError_Inches", strafe * 12);
        Logger.recordOutput("Movement/RotationError_Degrees", rotation);
    }

    private double calculateDistanceFromArea(double area) {
        return 12.23504 * Math.pow(0.999818, area) + 1.19735;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public double[] getTcornxy() {
        return tcornxy;
    }
    
    public int getTagId() {
        return tagId;
    }
    
    public double getCurrentDistance() {
        if (isCornersValid()) {
            return calculateDistanceFromArea(calculateTagArea());
        }
        return -1.0;
    }
    
    public boolean isAligned() {
        if (!isCornersValid()) return false;
        
        double currentDistance = getCurrentDistance();
        double forwardError = currentDistance - TARGET_DISTANCE_FEET;
        double rotationError = tx;
        
        return checkAlignment(forwardError, rotationError);
    }

    public Pose2d getEstimatedPose() {
        if (aprilTagFieldLayout == null || tagId == -1 || !isCornersValid()) {
            return null;
        }

        Optional<edu.wpi.first.math.geometry.Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
        if (!tagPoseOptional.isPresent()) {
            return null;
        }

        edu.wpi.first.math.geometry.Pose3d tagPose3d = tagPoseOptional.get();
        Pose2d tagPose = tagPose3d.toPose2d();
        
        double realTimeTA = calculateTagArea();
        double cameraDistanceToTag = calculateDistanceFromArea(realTimeTA);
        
        double cameraOffsetDistance = Math.sqrt(
            Math.pow(CAMERA_OFFSET_X_INCHES / 12.0, 2) + 
            Math.pow(CAMERA_OFFSET_Y_INCHES / 12.0, 2)
        );
        
        double horizontalAngleError = tx;
        
        double adjustedCameraDistance = cameraDistanceToTag / Math.cos(Math.toRadians(CAMERA_MOUNT_ANGLE_Y_DEGREES));
        
        double robotCenterDistanceToTag = Math.sqrt(
            Math.pow(adjustedCameraDistance, 2) + Math.pow(cameraOffsetDistance, 2) - 
            2 * adjustedCameraDistance * cameraOffsetDistance * 
            Math.cos(Math.toRadians(horizontalAngleError + CAMERA_MOUNT_ANGLE_X_DEGREES))
        );
        
        double currentDistance = robotCenterDistanceToTag + 1.0;
        double distanceMeters = currentDistance * 0.3048;
        
        double robotAngleToTag = tagPose.getRotation().getDegrees() + 180 - horizontalAngleError;
        Rotation2d robotRotation = Rotation2d.fromDegrees(robotAngleToTag);
        
        double angleRad = Math.toRadians(robotAngleToTag);
        double robotX = tagPose.getX() - distanceMeters * Math.cos(angleRad);
        double robotY = tagPose.getY() - distanceMeters * Math.sin(angleRad);
        
        Translation2d robotTranslation = new Translation2d(robotX, robotY);
        return new Pose2d(robotTranslation, robotRotation);
    }
    
    public double getTimestamp() {
        return Timer.getFPGATimestamp();
    }
}