package org.firstinspires.ftc.teamcode.robot;

//TODO: localization issue with weird values, exposure tuning, calibration (with scrcpy?)

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Contains April Tag Processing for Robot Localization
 * and Object Detection for Auto
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
        aprilTagProcessor = new AprilTagProcessor.Builder().build(); //TODO: calibration???
        //TODO: double check
        VisionPortal frontCamPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(1280, 720)) //TODO: double check
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        VisionPortal backCamPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        detections = new ArrayList<>();
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
                        aprilTagDetection.ftcPose.x, //TODO: raw pose???
                        aprilTagDetection.ftcPose.y,
                        aprilTagDetection.ftcPose.yaw,
                        aprilTagDetection.id }
                );
            }
        }
    }

    /**
     * Localizes the Robot
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
     * Returns the number of April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return detections.size();
    }
}
