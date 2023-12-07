package org.firstinspires.ftc.teamcode.robot;

//TODO: switching cameras, exposure tuning (with scrcpy?)

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * The Vision Subsystem on the Robot
 */
public class Vision implements Constants {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final WebcamName webcam1, webcam2;
    private ArrayList<AprilTagDetection> currentDetections;
    private int nullTags;
    private boolean frontCamIsActive;

    /**
     * Initializes the Robot Vision
     *
     * @param hwMap the Hardware Map
     */
    public Vision(HardwareMap hwMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        webcam1 = hwMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hwMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        currentDetections = new ArrayList<>();
        nullTags = 0;
        frontCamIsActive = true;
    }

    /**
     * Localizes the Robot
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        updateDetections();

        int size = currentDetections.size();

        double[] averagedPose = new double[3];
        for(int i = 0; i < size; i++) {
            double[] pose = localize(i);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] += pose[2];
        }

        if(size == 0 || size == nullTags) {
            switchCameras();
            return null;
        }

        averagedPose[0] /= size - nullTags;
        averagedPose[1] /= size - nullTags;
        averagedPose[2] /= size - nullTags;

        switchCameras();
        return averagedPose;
    }

    private void updateDetections() {
        currentDetections = aprilTagProcessor.getDetections();
        nullTags = 0;
    }

    private double[] localize(int i) {
        AprilTagDetection aprilTagDetection = currentDetections.get(i);

        if (aprilTagDetection.metadata == null) {
            nullTags++;
            return new double[]{0.0, 0.0, 0.0};
        }

        int id = aprilTagDetection.id;
        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;

        double tagX = (id >= 7 && id <= 10) ? APRIL_TAG_INTAKE_X : APRIL_TAG_SCORING_X;
        double tagY = getTagYCoordinate(id);

        double fieldHeading = frontCamIsActive == (id >= 7 && id <= 10) ? -yaw : MathUtilities.addAngles(-yaw, -180.0);

        double cameraDeltaX = range * Math.cos(Math.toRadians(bearing - yaw));
        double cameraDeltaY = range * Math.sin(Math.toRadians(bearing - yaw));

        double cameraX = (id >= 7 && id <= 10) ? tagX - cameraDeltaX : tagX + cameraDeltaX;
        double cameraY = (id >= 7 && id <= 10) ? tagY - cameraDeltaY : tagY + cameraDeltaY;

        double fieldHeadingInRadians = Math.toRadians(fieldHeading);

        double localizedX;
        double localizedY;
        if(frontCamIsActive) {
            localizedX = cameraX - FRONT_CAM_FORWARD_DIST * Math.cos(fieldHeadingInRadians)
                    + FRONT_CAM_HORIZONTAL_DIST * Math.sin(fieldHeadingInRadians);
            localizedY = cameraY - FRONT_CAM_HORIZONTAL_DIST * Math.cos(fieldHeadingInRadians)
                    - FRONT_CAM_FORWARD_DIST * Math.sin(fieldHeadingInRadians);
        }
        else {
            localizedX = cameraX + BACK_CAM_DIST * Math.cos(fieldHeadingInRadians);
            localizedY = cameraY + BACK_CAM_DIST * Math.sin(fieldHeadingInRadians);
        }

        return new double[] {localizedX, localizedY, fieldHeading};
    }

    private double getTagYCoordinate(int id) {
        switch(id) {
            case 1:
                return APRIL_TAG_1_Y;
            case 2:
                return APRIL_TAG_2_Y;
            case 3:
                return APRIL_TAG_3_Y;
            case 4:
                return APRIL_TAG_4_Y;
            case 5:
                return APRIL_TAG_5_Y;
            case 6:
                return APRIL_TAG_6_Y;
            case 7:
                return APRIL_TAG_7_Y;
            case 8:
                return APRIL_TAG_8_Y;
            case 9:
                return APRIL_TAG_9_Y;
            default:
                return APRIL_TAG_10_Y;
        }
    }

    private void switchCameras() {
        if(currentDetections.size() == 0 || currentDetections.size() == nullTags)
            frontCamIsActive = !frontCamIsActive;
        visionPortal.setActiveCamera(frontCamIsActive ? webcam1 : webcam2 );
    }

    /**
     * Returns the number of April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return currentDetections.size();
    }

    /**
     * Closes the vision portal
     */
    public void close() {
        visionPortal.close();
    }
}
