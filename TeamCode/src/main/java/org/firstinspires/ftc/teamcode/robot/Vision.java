package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//TODO: heading is still actually fricked

/**
 * The Vision Subsystem of the Robot
 */
public class Vision implements Constants {

    VisionPortal.Builder visionPortalBuilder;
    int FRONT_CAM_VIEW_ID;
    int BACK_CAM_VIEW_ID;
    AprilTagProcessor frontAprilTagProcessor;
    AprilTagProcessor backAprilTagProcessor;
    VisionPortal frontVisionPortal;
    VisionPortal backVisionPortal;
    ArrayList<AprilTagDetection> frontDetections;
    ArrayList<AprilTagDetection> backDetections;

    /**
     * Instantiates the Vision Subsystem
     *
     * @param hwMap the hardware map
     */
    public Vision(HardwareMap hwMap) {
        List<Integer> myPortalsList;
        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        FRONT_CAM_VIEW_ID = (Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false);
        BACK_CAM_VIEW_ID = (Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false);

        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        frontAprilTagProcessor = myAprilTagProcessorBuilder.build();
        backAprilTagProcessor = myAprilTagProcessorBuilder.build();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortalBuilder.addProcessor(frontAprilTagProcessor);
        visionPortalBuilder.setLiveViewContainerId(FRONT_CAM_VIEW_ID);
        frontVisionPortal = visionPortalBuilder.build();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 2"));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortalBuilder.addProcessor(backAprilTagProcessor);
        visionPortalBuilder.setLiveViewContainerId(BACK_CAM_VIEW_ID);
        backVisionPortal = visionPortalBuilder.build();
    }

    /**
     * Updates the Robot Vision, call in each loop
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        updateDetections();

        double[] averagedPose = new double[3];

        int frontSize = frontDetections.size();
        for(int i = 0; i < frontSize; i++) {
            double[] pose = localize(i, true);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] = MathUtilities.addAnglesForAverage(averagedPose[2], pose[2]);
        }

        int backSize = backDetections.size();
        for(int i = 0; i < backSize; i++) {
            double[] pose = localize(i, false);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] = MathUtilities.addAnglesForAverage(averagedPose[2], pose[2]);
        }

        if(frontSize + backSize == 0)
            return null;

        averagedPose[0] /= frontSize + backSize;
        averagedPose[1] /= frontSize + backSize;
        averagedPose[2] /= frontSize + backSize;

        return averagedPose;
    }

    private void updateDetections() {
        frontDetections = frontAprilTagProcessor.getDetections();
        backDetections = backAprilTagProcessor.getDetections();
    }

    private double[] localize(int i, boolean isFront) {
        AprilTagDetection aprilTagDetection = isFront ? frontDetections.get(i) : backDetections.get(i);

        int id = aprilTagDetection.id;
        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;

        double tagX = (id >= 7 && id <= 10) ? APRIL_TAG_INTAKE_X : APRIL_TAG_SCORING_X;
        double tagY = getTagYCoordinate(id);

        double fieldHeading = isFront == (id >= 7 && id <= 10) ? -yaw : MathUtilities.addAngles(-yaw, -180.0);

        double cameraDeltaX = range * Math.cos(Math.toRadians(bearing - yaw));
        double cameraDeltaY = range * Math.sin(Math.toRadians(bearing - yaw));

        double cameraX = (id >= 7 && id <= 10) ? tagX - cameraDeltaX : tagX + cameraDeltaX;
        double cameraY = (id >= 7 && id <= 10) ? tagY - cameraDeltaY : tagY + cameraDeltaY;

        double fieldHeadingInRadians = Math.toRadians(fieldHeading);

        double localizedX;
        double localizedY;
        if(isFront) {
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

    /**
     * Returns the number of April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return frontDetections.size() + backDetections.size();
    }
}
