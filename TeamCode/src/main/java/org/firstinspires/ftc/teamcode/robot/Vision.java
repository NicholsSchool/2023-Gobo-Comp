package org.firstinspires.ftc.teamcode.robot;

//TODO: parallel cameras, localization, exposure tuning (with scrcpy?)

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * The Vision Subsystem on the Robot
 */
public class Vision {
    private final AprilTagProcessor aprilTagProcessor;
    private final ArrayList<double[]> detections;

    /**
     * Initializes the Robot Vision
     *
     * @param hwMap the Hardware Map
     */
    public Vision(HardwareMap hwMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal frontCamPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        VisionPortal backCamPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        detections = new ArrayList<>();
    }

    /**
     * Localizes the Robot
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] localize() {
        updateDetections();

        int size = detections.size();
        if(size == 0)
            return null;

        double[] averagedPose = new double[3];

        for(int i = 0; i < size; i++)
        {
            int id = (int)detections.get(i)[0];
            boolean isFrontTag = id >= 7 && id <= 10;
            if(isFrontTag)
            {
                //TODO: intake side localization calculations
                //detections[i][1 or 2 or 3] = ...
            }
            else
            {
                //TODO: scoring side localization calculations
            }
            averagedPose[0] += detections.get(i)[0];
            averagedPose[1] += detections.get(i)[1];
            averagedPose[2] += detections.get(i)[2];
        }
        averagedPose[0] /= size;
        averagedPose[1] /= size;
        averagedPose[2] /= size;

        return averagedPose;
    }

    /**
     * Iterates through the April Tag Detections on both Vision Portals
     */
    public void updateDetections() {
        detections.clear();
        ArrayList<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        for(AprilTagDetection aprilTagDetection : aprilTagDetections) {
            if (aprilTagDetection.metadata != null) {
                detections.add( new double[] {
                        aprilTagDetection.ftcPose.x, //TODO: localize
                        aprilTagDetection.ftcPose.y,
                        aprilTagDetection.ftcPose.yaw,
                        aprilTagDetection.id }
                );
            }
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
