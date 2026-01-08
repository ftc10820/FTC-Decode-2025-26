package org.firstinspires.ftc.teamcode.huskylens;

import static android.os.SystemClock.sleep;
import static java.lang.Math.tan;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.threeten.bp.LocalTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

public class HuskyLensCam {
    private static final double CM_TO_IN = 0.393701;

    // === Camera intrinsic parameters ===
    private static final double HORIZONTAL_FOV = 78.5; // degrees
    private static final double VERTICAL_FOV = 65.5;   // degrees
    private static final int CAMERA_CENTER_X = 160;    // pixels
    private static final int CAMERA_CENTER_Y = 120;    // pixels
    private static final double VERTICAL_FOCAL_LENGTH =
            CAMERA_CENTER_Y / tan(Math.toRadians(VERTICAL_FOV / 2.0)); // pixels

    // === Instance fields ===
    private volatile int specimenScreenYLimit;
    private volatile double focalPoint;
    private HuskyLens camera;

    // Camera mounting parameters
    private volatile double cameraHeightCm; // height from ground (cm)
    private volatile double cameraTiltDeg;  // downward = negative, upward = positive
    private volatile double cameraLateralOffsetCm; // offset from robot center (cm), negative is left, positive is right

    private final ConcurrentHashMap<Integer, String> idToColor = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, Double> widthOfGameElements = new ConcurrentHashMap<>();

    // === Constructor ===
    /**
     * @param camera HuskyLens instance
     * @param focalPoint initial focal length (px)
     * @param specimenScreenYLimit y limit for color detection
     * @param cameraHeightCm camera height from ground (cm)
     * @param cameraTiltDeg camera tilt (degrees, downward negative)
     */
    public HuskyLensCam(HuskyLens camera,
                        double focalPoint,
                        int specimenScreenYLimit,
                        double cameraHeightCm,
                        double cameraTiltDeg) {
        this(camera, focalPoint, specimenScreenYLimit, cameraHeightCm, cameraTiltDeg, 0.0);
    }

    /**
     * @param camera HuskyLens instance
     * @param focalPoint initial focal length (px)
     * @param specimenScreenYLimit y limit for color detection
     * @param cameraHeightCm camera height from ground (cm)
     * @param cameraTiltDeg camera tilt (degrees, downward negative)
     * @param cameraLateralOffsetCm camera lateral offset from robot center (cm), negative left, positive right
     */
    public HuskyLensCam(HuskyLens camera,
                        double focalPoint,
                        int specimenScreenYLimit,
                        double cameraHeightCm,
                        double cameraTiltDeg,
                        double cameraLateralOffsetCm) {
        if (camera == null) {
            throw new NullPointerException("INIT: Camera is null");
        }
        this.camera = camera;
        this.focalPoint = focalPoint;
        this.specimenScreenYLimit = specimenScreenYLimit;
        this.cameraHeightCm = cameraHeightCm;
        this.cameraTiltDeg = cameraTiltDeg;
        this.cameraLateralOffsetCm = cameraLateralOffsetCm;

        // Define color IDs
        idToColor.put(1, "Yellow");
        idToColor.put(2, "Blue");
        idToColor.put(3, "Red");

        // Real-world object widths (cm)
        widthOfGameElements.put("specimen", 4.25);
        widthOfGameElements.put("apriltag", 16.51);
    }

    // === Getters ===
    public double getFocalPoint() { return focalPoint; }
    public int getSpecimenScreenYLimit() { return specimenScreenYLimit; }
    public double getCameraHeightCm() { return cameraHeightCm; }
    public double getCameraTiltDeg() { return cameraTiltDeg; }
    public double getCameraLateralOffsetCm() { return cameraLateralOffsetCm; }
    public HuskyLens getCamera() { return camera; }

    private void setFocalPoint(double focalPoint) { this.focalPoint = focalPoint; }

    // === Core Computation Helper ===
    private ObjectInfo processBlock(HuskyLens.Block block, double realWidth, String type, int id) {
        LocalTime time = LocalTime.now();

        // Pixel offsets
        double xOffset = block.x - CAMERA_CENTER_X;
        double yOffset = block.y - CAMERA_CENTER_Y; // yOffset is used later for pitch

        // Horizontal angle (yaw)
        double yawAngleDeg = (xOffset / CAMERA_CENTER_X) * (HORIZONTAL_FOV / 2.0);

        // ... (rawPitchDeg, pitchAngleDeg calculations omitted for brevity, assumed unchanged)
        // Raw pitch angle (positive = up, negative = down)
        double rawPitchDeg = -(yOffset / CAMERA_CENTER_Y) * (VERTICAL_FOV / 2.0);
        // Adjust with camera tilt
        double pitchAngleDeg = rawPitchDeg + cameraTiltDeg;

        // Corrected pixel width
        double yawRad = Math.toRadians(yawAngleDeg);
        double correctedPixelWidth = block.width / Math.cos(yawRad);

        // Compute horizontal distance (straight line distance to object)
        double distance = (realWidth * focalPoint) / correctedPixelWidth;

        // --- NEW CALCULATION ---
        // Compute the lateral distance (left/right offset from the center of the camera)
        double lateralDistanceFromCamera = Math.sin(yawRad) * distance;
        // Adjust for the camera's lateral offset from the robot's center
        double lateralDistanceFromRobotCenter = lateralDistanceFromCamera + this.cameraLateralOffsetCm;
        // -----------------------

        // Compute vertical offset relative to camera
        double totalPitchRad = Math.toRadians(pitchAngleDeg);
        double verticalOffset = Math.tan(totalPitchRad) * distance;

        // Estimate object height from ground
        double objectHeight = cameraHeightCm + verticalOffset;

        return new ObjectInfo(
                block.x, block.y,
                type, id,
                distance,
                lateralDistanceFromRobotCenter, // <--- Pass the new value here
                yawAngleDeg,
                pitchAngleDeg,
                objectHeight,
                time
        );
    }



    // === Color Recognition ===
    public List<ObjectInfo> scanColor() {
        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        HuskyLens camera = getCamera();

        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        sleep(100);

        HuskyLens.Block[] colorBlocks = camera.blocks();
        if (colorBlocks == null || colorBlocks.length == 0) return objects;

        for (HuskyLens.Block block : colorBlocks) {

            if (block.y <= getSpecimenScreenYLimit()) {
                double realWidth = widthOfGameElements.get("specimen");

                objects.add(processBlock(block, realWidth, "color", block.id));
            }
        }
        return objects;
    }

    // === AprilTag R
    // ecognition ===
    public List<ObjectInfo> scanTag() {
        List<ObjectInfo> objects = Collections.synchronizedList(new ArrayList<>());
        HuskyLens camera = getCamera();

        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(200);

        HuskyLens.Block[] apriltagBlocks = camera.blocks();
        if (apriltagBlocks == null || apriltagBlocks.length == 0) return objects;

        for (HuskyLens.Block block : apriltagBlocks) {
            double realWidth = widthOfGameElements.get("apriltag");
            objects.add(processBlock(block, realWidth, "apriltag", block.id));
        }
        return objects;
    }

    // === Combined Scan ===
    public List<ObjectInfo> scanAll() {
        List<ObjectInfo> allObjects = Collections.synchronizedList(new ArrayList<>());
        allObjects.addAll(scanColor());
        sleep(1000);
        allObjects.addAll(scanTag());
        return allObjects;
    }
    public Pose2d getPoseOf(Pose2d currentPose, ObjectInfo object) {
        double objectYawRad = Math.toRadians(object.yaw);


        double forwardDistanceCm = object.distance * Math.cos(objectYawRad);

        // This is the object's lateral distance from the ROBOT's CENTER, already adjusted for camera offset.
        double lateralDistanceFromRobotCenterCm = object.lateralDistance;


        double forwardDistanceIn = forwardDistanceCm * CM_TO_IN;
        double lateralDistanceIn = lateralDistanceFromRobotCenterCm * CM_TO_IN;


        double robotRelativeX = forwardDistanceIn;
        double robotRelativeY = -lateralDistanceIn;


        double robotHeadingRad = currentPose.heading.log();


        double fieldOffsetX = robotRelativeX * Math.cos(robotHeadingRad) - robotRelativeY * Math.sin(robotHeadingRad);
        double fieldOffsetY = robotRelativeX * Math.sin(robotHeadingRad) + robotRelativeY * Math.cos(robotHeadingRad);


        double objectFieldX = currentPose.position.x + fieldOffsetX;
        double objectFieldY = currentPose.position.y + fieldOffsetY;

        double headingToObject = Math.atan2(fieldOffsetY, fieldOffsetX);

        return new Pose2d(new Vector2d(objectFieldX, objectFieldY), headingToObject);
    }


    // === Auto Tuning (Downward Tilt Negative) ===
    public double autoTuneFocalLength(LinearOpMode opMode,
                                      double knownDistanceCm,
                                      double knownHeightCm,
                                      String objectType,
                                      int averageCount) {

        HuskyLens camera = getCamera();
        double realWidth = widthOfGameElements.getOrDefault(objectType, 0.0);

        if (realWidth == 0.0) {
            opMode.telemetry.addLine("❌ Unknown object type for tuning!");
            opMode.telemetry.update();
            return getFocalPoint();
        }

        opMode.telemetry.addLine("Auto-tuning focal length (downward = negative)...");
        opMode.telemetry.addData("Camera tilt (deg)", cameraTiltDeg);
        opMode.telemetry.addData("Camera height (cm)", cameraHeightCm);
        opMode.telemetry.addData("Target height (cm)", knownHeightCm);
        opMode.telemetry.addData("Horizontal distance (cm)", knownDistanceCm);
        opMode.telemetry.update();

        camera.selectAlgorithm(
                objectType.equals("apriltag") ?
                        HuskyLens.Algorithm.TAG_RECOGNITION :
                        HuskyLens.Algorithm.COLOR_RECOGNITION
        );

        sleep(500);
        List<Double> focalSamples = new ArrayList<>();

        while (opMode.opModeIsActive() && focalSamples.size() < averageCount) {
            HuskyLens.Block[] blocks = camera.blocks();
            if (blocks != null && blocks.length > 0) {
                HuskyLens.Block block = blocks[0];
                double pixelWidth = block.width;

                if (pixelWidth > 0) {
                    double verticalDiff = knownHeightCm - cameraHeightCm;
                    double slantDistance = Math.sqrt(
                            knownDistanceCm * knownDistanceCm +
                                    verticalDiff * verticalDiff
                    );

                    double f = (pixelWidth * slantDistance) / realWidth;
                    focalSamples.add(f);

                    opMode.telemetry.addData("Sample", focalSamples.size());
                    opMode.telemetry.addData("Pixel width", "%.2f", pixelWidth);
                    opMode.telemetry.addData("Instant focal", "%.2f", f);
                    opMode.telemetry.update();
                }
            } else {
                opMode.telemetry.addLine("No target detected...");
                opMode.telemetry.update();
            }
            sleep(100);
        }

        if (focalSamples.isEmpty()) {
            opMode.telemetry.addLine("❌ Calibration failed: no samples.");
            opMode.telemetry.update();
            return getFocalPoint();
        }

        double sum = 0.0;
        for (double f : focalSamples) sum += f;
        double avgFocal = sum / focalSamples.size();

        setFocalPoint(avgFocal);

        opMode.telemetry.addLine("✅ Tilt + height-aware tuning complete!");
        opMode.telemetry.addData("New focal length (px)", "%.2f", avgFocal);
        opMode.telemetry.update();

        return avgFocal;
    }
}
