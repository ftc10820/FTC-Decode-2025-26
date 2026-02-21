package org.firstinspires.ftc.teamcode.camera.limelight;

import static android.os.SystemClock.sleep;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.camera.Camera;
import org.firstinspires.ftc.teamcode.camera.ObjectInfo;
import org.threeten.bp.LocalTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;


public class LimelightCam implements Camera {
    private static final double CM_TO_IN = 0.393701;

    // === Instance fields ===
    private final Limelight3A camera; // Instance of your Limelight driver class

    // Camera mounting parameters
    private final double cameraHeightCm; // height from ground (cm)
    private final double cameraTiltDeg;  // downward = negative, upward = positive
    private final double cameraLateralOffsetCm; // offset from robot center (cm), negative is left, positive is right

    private final ConcurrentHashMap<String, Double> heightOfGameElements = new ConcurrentHashMap<>();

    /**
     * Constructor for the Limelight camera adapter.
     * @param camera Your Limelight instance
     * @param cameraHeightCm Camera height from the ground (cm)
     * @param cameraTiltDeg Camera tilt angle in degrees (downward is negative)
     * @param cameraLateralOffsetCm Lateral offset from robot center (cm), negative is left
     */
    public LimelightCam(Limelight3A camera,
                        double cameraHeightCm,
                        double cameraTiltDeg,
                        double cameraLateralOffsetCm) {
        if (camera == null) {
            throw new NullPointerException("INIT: Limelight Camera is null");
        }
        this.camera = camera;
        camera.start();
        this.cameraHeightCm = cameraHeightCm;
        this.cameraTiltDeg = cameraTiltDeg;
        this.cameraLateralOffsetCm = cameraLateralOffsetCm;


        heightOfGameElements.put("ball-trio", 12.7); // Example height in cm
        heightOfGameElements.put("apriltag", 15.24); // Example height for AprilTag centers in cm
    }

    // === Getters ===
    public Limelight3A getCamera() { return camera; }

    /**
     * Processes a single Limelight target to calculate its position.
     * @param target The raw target data from the Limelight (FiducialResult or DetectorResult).
     * @param targetHeightCm The known real-world height of the target from the ground.
     * @param type A string identifier for the object type (e.g., "color", "apriltag").
     * @return An ObjectInfo instance with calculated positional data.
     */
    private ObjectInfo processTarget(Object target, double targetHeightCm, String type) {
        LocalTime time = LocalTime.now();
        double totalDistance, lateralDistance, robotYawDeg, totalPitchDeg;
        int id;
        double targetXPixels, targetYPixels;

        if (target instanceof LLResultTypes.FiducialResult) {
            LLResultTypes.FiducialResult fiducial = (LLResultTypes.FiducialResult) target;
            id = fiducial.getFiducialId();
            targetXPixels = fiducial.getTargetXPixels();
            targetYPixels = fiducial.getTargetYPixels();

            Pose3D pose = fiducial.getTargetPoseRobotSpace();
            lateralDistance = pose.getPosition().y * 100.0;

            double norm = Math.sqrt(Math.pow(pose.getPosition().x, 2) + Math.pow(pose.getPosition().y, 2) + Math.pow(pose.getPosition().z, 2));
            totalDistance = norm * 100.0;
            robotYawDeg = fiducial.getTargetXDegrees();
            totalPitchDeg = (double) pose.getOrientation().getPitch();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("non corrected distance",totalDistance);
            packet.put("corrected distance",totalDistance*1.376652+45);
        }
        else if (target instanceof LLResultTypes.DetectorResult) {
            LLResultTypes.DetectorResult detector = (LLResultTypes.DetectorResult) target;

            // Detector classes use ClassId; target pixels are at the center of the box
            id = (int) detector.getClassId();
            targetXPixels = 0; // DetectorResult doesn't expose raw pixels easily, usually tx/ty is preferred
            targetYPixels = 0;

            // Manual pose calculation using trigonometry
            double cameraTiltRad = Math.toRadians(cameraTiltDeg);
            double verticalAngleRad = Math.toRadians(detector.getTargetYDegrees());
            double horizontalAngleRad = Math.toRadians(detector.getTargetXDegrees());

            // Calculate forward distance on the ground plane
            double heightDifference = targetHeightCm - cameraHeightCm;
            double forwardDistanceCm = heightDifference / Math.tan(cameraTiltRad + verticalAngleRad);

            // Calculate lateral offset from the camera's center line
            double lateralOffsetFromCamAxis = forwardDistanceCm * Math.tan(horizontalAngleRad);
            lateralDistance = lateralOffsetFromCamAxis + cameraLateralOffsetCm;

            // Pythagorean theorem for total straight-line distance
            totalDistance = Math.sqrt(Math.pow(forwardDistanceCm, 2) + Math.pow(lateralOffsetFromCamAxis, 2) + Math.pow(heightDifference, 2));

            robotYawDeg = detector.getTargetXDegrees();
            totalPitchDeg = detector.getTargetYDegrees();
        }
        else {
            throw new IllegalArgumentException("Unsupported target: " + target.getClass().getSimpleName());
        }

        return new ObjectInfo(
                (int) targetXPixels, (int) targetYPixels,
                type, id,
                totalDistance*1.376652+49,
                lateralDistance,
                robotYawDeg,
                totalPitchDeg,
                targetHeightCm,
                time
        );
    }




    /**
     * Scans for AprilTag objects using a specific Limelight pipeline.
     * Assumes pipeline 0 is configured for AprilTag detection.
     * @return A list of detected and processed objects.
     */
    public List<ObjectInfo> scanTag() {
        final int pipelineIndex = 0; // ASSUMPTION: Pipeline 0 is for AprilTags
        final String objectType = "apriltag";

        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        Limelight3A cam = getCamera();

        Double realHeight = heightOfGameElements.get(objectType);
        if (realHeight == null) {
            throw new IllegalArgumentException("Height for object type '" + objectType + "' is not defined.");
        }

        cam.pipelineSwitch(pipelineIndex);
        sleep(50); // Small delay to allow pipeline to switch and process

        LLResult detectedResults = cam.getLatestResult();
        if (!detectedResults.isValid() || detectedResults.getFiducialResults() == null || detectedResults.getFiducialResults().isEmpty()) {
            return objects;
        }

        for (LLResultTypes.FiducialResult target : detectedResults.getFiducialResults()) {
            objects.add(processTarget(target, realHeight, objectType));
        }
        return objects;
    }

    /**
     * Scans for specimen objects using a specific Limelight pipeline.
     * Assumes pipeline 1 is configured for specimen detection.
     * @return A list of detected and processed objects.
     */
    public List<ObjectInfo> scanColor() {
        final int pipelineIndex = 2;
        final String objectType = "ball-trio";

        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        Limelight3A cam = getCamera();

        Double realHeight = heightOfGameElements.get(objectType);
        if (realHeight == null) {
            throw new IllegalArgumentException("Height for object type '" + objectType + "' is not defined.");
        }

        cam.pipelineSwitch(pipelineIndex);
        sleep(50);

        LLResult detectedResults = cam.getLatestResult();
        if (!detectedResults.isValid() || detectedResults.getDetectorResults() == null || detectedResults.getDetectorResults().isEmpty()) {
            return objects;
        }

        for (LLResultTypes.DetectorResult target : detectedResults.getDetectorResults()) {
            objects.add(processTarget(target, realHeight, objectType));
        }
        return objects;
    }

    public List<ObjectInfo> scanAll() {
        List<ObjectInfo> allObjects = Collections.synchronizedList(new ArrayList<>());
        allObjects.addAll(scanTag());
        allObjects.addAll(scanColor());
        return allObjects;
    }

    /**
     * Calculates the field-relative Pose2d of a detected object.
     * @param currentPose The current field-relative pose of the robot.
     * @param object The ObjectInfo data for the detected object.
     * @return A Pose2d representing the object's position on the field.
     */
    public Pose2d getPoseOf(Pose2d currentPose, ObjectInfo object) {
        double objectYawRad = Math.toRadians(object.yaw);

        // Use the forward and lateral distances calculated relative to the robot's center.
        double forwardDistanceCm = object.distance * Math.cos(objectYawRad);
        double lateralDistanceFromRobotCenterCm = object.lateralDistance;

        // Convert to inches for RoadRunner's Pose2d
        double forwardDistanceIn = forwardDistanceCm * CM_TO_IN;
        double lateralDistanceIn = lateralDistanceFromRobotCenterCm * CM_TO_IN;

        // Create a vector representing the object's position relative to the robot.
        // Y is inverted because in RoadRunner, positive Y is to the robot's left.
        double robotRelativeY = -lateralDistanceIn;

        // Rotate the relative vector by the robot's current heading.
        double robotHeadingRad = currentPose.heading.log();
        double fieldOffsetX = forwardDistanceIn * Math.cos(robotHeadingRad) - robotRelativeY * Math.sin(robotHeadingRad);
        double fieldOffsetY = forwardDistanceIn * Math.sin(robotHeadingRad) + robotRelativeY * Math.cos(robotHeadingRad);

        // Add the rotated offset to the robot's current field position.
        double objectFieldX = currentPose.position.x + fieldOffsetX;
        double objectFieldY = currentPose.position.y + fieldOffsetY;

        double headingToObject = Math.atan2(fieldOffsetY, fieldOffsetX);

        return new Pose2d(new Vector2d(objectFieldX, objectFieldY), headingToObject);
    }
}
