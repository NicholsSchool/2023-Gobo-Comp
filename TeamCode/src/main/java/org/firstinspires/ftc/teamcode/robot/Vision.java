package org.firstinspires.ftc.teamcode.robot;

//TODO: check localization, exposure tuning (with scrcpy?)

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
    private final ArrayList<double[]> detections;
    private boolean frontCamActive;

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

        detections = new ArrayList<>();
        frontCamActive = true;
    }

    /**
     * Localizes the Robot
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        updateDetections();

        int size = detections.size();

        if(size == 0)
            return null;

        double[] averagedPose = new double[3];

        for(int i = 0; i < size; i++) {
            double[] pose = localize(i);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] += pose[2];
        }
        averagedPose[0] /= size;
        averagedPose[1] /= size;
        averagedPose[2] /= size;

        switchCameras();

        return averagedPose;
    }

    private void updateDetections() {
        detections.clear();

        ArrayList<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        for(AprilTagDetection aprilTagDetection : aprilTagDetections) {
            if (aprilTagDetection.metadata != null) {
                detections.add( new double[] {
                        aprilTagDetection.id,
                        aprilTagDetection.ftcPose.range,
                        aprilTagDetection.ftcPose.yaw,
                        aprilTagDetection.ftcPose.bearing }
                );
            }
        }
    }

    private double[] localize(int i) {
        int id = (int)detections.get(i)[0];
        double tagX = (id >= 7 && id <= 10) ? APRIL_TAG_INTAKE_X : APRIL_TAG_SCORING_X;
        double tagY = 0.0;
        switch(id) {
            case 1:
                tagY = APRIL_TAG_1_Y;
                break;
            case 2:
                tagY = APRIL_TAG_2_Y;
                break;
            case 3:
                tagY = APRIL_TAG_3_Y;
                break;
            case 4:
                tagY = APRIL_TAG_4_Y;
                break;
            case 5:
                tagY = APRIL_TAG_5_Y;
                break;
            case 6:
                tagY = APRIL_TAG_6_Y;
                break;
            case 7:
                tagY = APRIL_TAG_7_Y;
                break;
            case 8:
                tagY = APRIL_TAG_8_Y;
                break;
            case 9:
                tagY = APRIL_TAG_9_Y;
                break;
            case 10:
                tagY = APRIL_TAG_10_Y;
                break;
        }

        double tagRange = detections.get(i)[1];
        double tagYaw = detections.get(i)[2];
        double tagBearing = detections.get(i)[3];

        double localizedHeading = frontCamActive == (id >= 7 && id <= 10) ? -tagYaw : MathUtilities.addAngles(-tagYaw, -180.0);

        double cameraDeltaX = tagRange * Math.cos(Math.toRadians(tagBearing - tagYaw));
        double cameraDeltaY = tagRange * Math.sin(Math.toRadians(tagBearing - tagYaw));

        double cameraX = (id >= 7 && id <= 10) ? tagX - cameraDeltaX : tagX + cameraDeltaX;
        double cameraY = (id >= 7 && id <= 10) ? tagY - cameraDeltaY : tagY + cameraDeltaY;

        double headingRadians = Math.toRadians(localizedHeading);
        double localizedX;
        double localizedY;
        if(frontCamActive) {
            localizedX = cameraX - FRONT_CAM_FORWARD_DIST * Math.cos(headingRadians)
                    + FRONT_CAM_HORIZONTAL_DIST * Math.sin(headingRadians);
            localizedY = cameraY - FRONT_CAM_HORIZONTAL_DIST * Math.cos(headingRadians)
                    - FRONT_CAM_FORWARD_DIST * Math.sin(headingRadians);
            }
        else {
            localizedX = cameraX + BACK_CAM_DIST * Math.cos(headingRadians);
            localizedY = cameraY + BACK_CAM_DIST * Math.sin(headingRadians);
        }

        return new double[] {localizedX, localizedY, localizedHeading};
    }

    private void switchCameras() {
        if(detections.size() == 0)
            frontCamActive = !frontCamActive;
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(frontCamActive ? webcam1 : webcam2 );
        }
    }

    /**
     * Returns the number of April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return detections.size();
    }
}
