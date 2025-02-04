package frc.robot.subsystems.VisionSubsystem;

import java.util.ArrayList;

import org.lasarobotics.vision.AprilTagCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    public static record Hardware(
            AprilTagCamera frontLeftCamera,
            AprilTagCamera frontRightCamera,
            AprilTagCamera rearCamera) {
    }

    private static ArrayList<AprilTagCamera> s_cameras;

    public VisionSubsystem(Hardware hardware) {
        s_cameras = new ArrayList<AprilTagCamera>();
        s_cameras.add(hardware.frontLeftCamera);
        s_cameras.add(hardware.frontRightCamera);
        s_cameras.add(hardware.rearCamera);
    }

    public static ArrayList<AprilTagCamera> getAprilTagCameras() {
        return s_cameras;
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

        Hardware hardware = new Hardware(
                frontLeftCamera,
                frontRightCamera,
                rearCamera);
        return hardware;
    }

    @Override
    public void periodic() {

    }
}
