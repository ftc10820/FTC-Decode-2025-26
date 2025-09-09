package org.firstinspires.ftc.teamcode.huskylens;
import static android.os.SystemClock.sleep;


import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.threeten.bp.LocalTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

public class HuskyLensCam {
    //TODO: Add exception handling framework


    private static final double HORIZONTAL_FOV = 78.5; // degrees
    private static final double VERTICAL_FOV = 65.5;   // degrees
    private static final int CAMERA_CENTER_X = 160;   // pixels
    private static final int CAMERA_CENTER_Y = 120;   // pixels

    private int getSpecimenScreenYLimit() {
        return specimenScreenYLimit;
    }

    private void setSpecimenScreenYLimit(int specimenScreenYLimit) {
        this.specimenScreenYLimit = specimenScreenYLimit;
    }

    private volatile int specimenScreenYLimit;
    private ConcurrentHashMap<String, Double> getWidthOfGameElements() {
        return widthOfGameElements;
    }



    private ConcurrentHashMap<String,Double> widthOfGameElements = new ConcurrentHashMap<>();
    private double getFocalPoint() {
        return focalPoint;
    }

    private ConcurrentHashMap<Integer, String> getIdToColor() {
        return idToColor;
    }



    private ConcurrentHashMap<Integer,String> idToColor = new ConcurrentHashMap<>();
    private void setFocalPoint(double focalPoint) {
        this.focalPoint = focalPoint;
    }

    private volatile double focalPoint;
    private HuskyLens getCamera() {
        return camera;
    }

    private void setCamera(HuskyLens camera) {
        this.camera = camera;
    }

    private HuskyLens camera = null;
    public HuskyLensCam(HuskyLens camera, double focalPoint, int specimenScreenYLimit){
        setCamera(camera);
        if (getCamera() == null){
            throw new NullPointerException("INIT: Camera is null");
        }
        setFocalPoint(focalPoint);
        setSpecimenScreenYLimit(specimenScreenYLimit);
        idToColor.put(1,"Yellow");
        idToColor.put(2,"Blue");
        idToColor.put(3,"Red");
        //TODO: add real width of elements
        widthOfGameElements.put("specimen",4.25);
        widthOfGameElements.put("apriltag",5.21);

    }
    public List<ObjectInfo> scanColor() {
        HuskyLens camera = getCamera();
        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        double focalPoint = getFocalPoint();
        //detecting specimen

        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        sleep(100);
        HuskyLens.Block[] colorBlocks = camera.blocks();
        int specimenScreenYLimit = getSpecimenScreenYLimit();

        for (HuskyLens.Block colorBlock : colorBlocks) {
            if (colorBlock.y <= specimenScreenYLimit) {
                double realWidth = widthOfGameElements.get("specimen");
                LocalTime time = LocalTime.now();


                double pixelOffsetX = colorBlock.x - CAMERA_CENTER_X;
                double yawAngleDegrees = (pixelOffsetX / CAMERA_CENTER_X) * (HORIZONTAL_FOV / 2.0);


                double pixelOffsetY = colorBlock.y - CAMERA_CENTER_Y;
                double pitchAngleDegrees = (pixelOffsetY / CAMERA_CENTER_Y) * (VERTICAL_FOV / 2.0);


                double yawAngleRadians = Math.toRadians(yawAngleDegrees);
                double correctedPixelWidth = colorBlock.width / Math.cos(yawAngleRadians);


                double correctedDistance = (realWidth * focalPoint) / correctedPixelWidth;

                objects.add(new ObjectInfo(colorBlock.x, colorBlock.y, "color", idToColor.get(colorBlock.id), correctedDistance, yawAngleDegrees, pitchAngleDegrees, time));
            }
        }
        return objects;
    }

    public List <ObjectInfo> scanTag(){
        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        HuskyLens camera = getCamera(); // Added to get camera instance
        double focalPoint = getFocalPoint(); // Added to get focal point
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(200);
        HuskyLens.Block[] apriltagBlocks = camera.blocks();


        for (HuskyLens.Block apriltagBlock : apriltagBlocks) {
            double realWidth = widthOfGameElements.get("apriltag");
            LocalTime time = LocalTime.now();

            // Calculate yaw (horizontal angle)
            double pixelOffsetX = apriltagBlock.x - CAMERA_CENTER_X;
            double yawAngleDegrees = (pixelOffsetX / CAMERA_CENTER_X) * (HORIZONTAL_FOV / 2.0);

            // Calculate pitch (vertical angle)
            double pixelOffsetY = apriltagBlock.y - CAMERA_CENTER_Y;
            double pitchAngleDegrees = (pixelOffsetY / CAMERA_CENTER_Y) * (VERTICAL_FOV / 2.0);

            // Compensate for yaw angle in distance calculation
            double yawAngleRadians = Math.toRadians(yawAngleDegrees);
            double correctedPixelWidth = apriltagBlock.width / Math.cos(yawAngleRadians);

            // Calculate corrected distance
            double correctedDistance = (realWidth * focalPoint) / correctedPixelWidth;

            objects.add(new ObjectInfo(apriltagBlock.x, apriltagBlock.y, "apriltag", "apriltag id #" + apriltagBlock.id, correctedDistance, yawAngleDegrees, pitchAngleDegrees, time));
        }
        return objects;
    }

    public List<ObjectInfo> scanAll() {
        //TODO: Improve logic
        // add exception handling
        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        objects.addAll(scanColor());
        sleep(1000);
        objects.addAll(scanTag());
        return objects;
    }
}
