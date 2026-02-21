package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.camera.Camera;
import org.firstinspires.ftc.teamcode.camera.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.camera.ObjectInfo;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class AutomationsActions {

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    public class Shooter {
        private final DcMotorEx motor;
        public final double TICKS_PER_REV = 28;
        public final double FLYWHEEL_RPM = 2500;


        public Shooter(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "flywheel");

        }

        public class SpinUp implements Action {
            private boolean initialized = false;
            private final double targetVelocity;

            public SpinUp(double rpm) {
                this.targetVelocity = TICKS_PER_REV * rpm / 60.0;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setDirection(DcMotorEx.Direction.REVERSE);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocity(targetVelocity);
                    initialized = true;
                }

                double vel = motor.getVelocity();
                packet.put("shooterVelocity goal", -targetVelocity);
                packet.put("shooterVelocity", vel);
                return (Math.abs(vel) < 0.95 * Math.abs(targetVelocity));
            }
        }

        public double getRPMFromDistance(double distanceInCm, double targetHeightInCm) {
            double R = distanceInCm / 100.0; // Distance (m)
            final double g = 9.81;

            double r_cm = DecodeConstants.FLYWHEEL_RADIUS_CM;
            double r = r_cm / 100.0; // Flywheel Radius (m)

            double theta_deg = DecodeConstants.LAUNCH_ANGLE_DEGREES;
            double theta = Math.toRadians(theta_deg); // Launch Angle (rad)

            double eta = DecodeConstants.SHOOTER_EFFICIENCY;

            double yt = targetHeightInCm / 100.0; // Target Height (m)

            double y0_cm = DecodeConstants.LAUNCH_HEIGHT_CM;
            double y0 = y0_cm / 100.0; // Launch Height (m)

            // RPM = (60 / (pi * r * eta)) * sqrt((g * R^2) / (2 * cos^2(theta) * (R * tan(theta) - (y_t - y_0))))
            double term1 = 60.0 / (Math.PI * r * eta);
            double cosTheta = Math.cos(theta);
            double tanTheta = Math.tan(theta);

            double numerator = g * R * R;
            double denominator = 2 * cosTheta * cosTheta * (R * tanTheta - (yt - y0));

            double sqrtTerm = Math.sqrt(numerator / denominator);

            return term1 * sqrtTerm;
        }

        public double getEfficiencyFactor(double currentRpm, double distanceInCm, double targetHeightInCm) {
            double R = distanceInCm / 100.0;
            double yt = targetHeightInCm / 100.0;
            final double g = 9.81;

            double r = DecodeConstants.FLYWHEEL_RADIUS_CM / 100.0;
            double theta = Math.toRadians(DecodeConstants.LAUNCH_ANGLE_DEGREES);
            double y0 = DecodeConstants.LAUNCH_HEIGHT_CM / 100.0;

            if (currentRpm == 0) {
                return 0.0;
            }

            double term1 = 60.0 / (Math.PI * r * currentRpm);

            double cosTheta = Math.cos(theta);
            double tanTheta = Math.tan(theta);

            double numerator = g * R * R;
            double denominator = 2 * cosTheta * cosTheta * (R * tanTheta - (yt - y0));

            if (denominator <= 0) {
                return 0.0;
            }

            double sqrtTerm = Math.sqrt(numerator / denominator);

            return term1 * sqrtTerm;
        }


        public double getRPMFromDistance(ObjectInfo objectInfo) {
            return getRPMFromDistance(objectInfo.distance, objectInfo.realHeight+48);
        }

        public Action spinUp(double rpm) {
            return new SpinUp(rpm);
        }

        public Action spinUp(ObjectInfo objectInfo) {
            return new SpinUp(getRPMFromDistance(objectInfo));
        }

        public Action spinUp() {
            return new SpinUp(FLYWHEEL_RPM);
        }

    }

    public class Intake{
        private final DcMotorEx intake;
        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        }
        public class IntakeAction implements Action {
            private boolean initialized = false;
            private final double power;
            public IntakeAction(double power) {
                this.power = power;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    initialized = true;

                }
                intake.setPower(power);
                return false;
            }
        }
        public Action intakeAction(double power) {
            return new IntakeAction(power);
        }


    }

    public class Transfer {
        private final Servo transfer1, transfer2, transfer3;
        private final ColorSensor colorSensor1, colorSensor2, colorSensor3;
        private final MecanumDrive drive;

        // The horizontal distance from the center of the robot to the left/right shooter shafts in cm.
        // This is the "opposite" side of our trigonometry triangle.
        // THIS VALUE NEEDS TO BE MEASURED AND TUNED ON YOUR ROBOT.
        private static final double SHOOTER_OFFSET_CM = 5.75;

        private static final double POS_INITIAL = 0.5;
        private static final double POS_ACTIVE = 0;
        private static final double WAIT_TIME = 1.0; // Time for servo to actuate and ball to be shot

        public Transfer(HardwareMap hardwareMap) {
            this(hardwareMap, null);
        }

        public Transfer(HardwareMap hardwareMap, MecanumDrive drive) {
            this.drive = drive;
            transfer1 = hardwareMap.get(Servo.class, "transfer");
            transfer2 = hardwareMap.get(Servo.class, "transfer2");
            transfer3 = hardwareMap.get(Servo.class, "transfer3");
            colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
            colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");

            transfer1.setPosition(POS_INITIAL);
            transfer2.setPosition(POS_INITIAL);
            transfer3.setPosition(POS_INITIAL);
        }

        public BallColor getColorOfSensor(int sensorId) {
            ColorSensor sensor;
            switch (sensorId) {
                case 1: sensor = colorSensor1; break;
                case 2: sensor = colorSensor2; break;
                case 3: sensor = colorSensor3; break;
                default: return BallColor.NONE;
            }

            double greenValue = Math.max(1, sensor.green());
            double redValue = Math.max(1, sensor.red());
            double blueValue = Math.max(1, sensor.blue());

            double totalColor = Math.max(1, redValue + greenValue + blueValue);
            double normalizedRed = (redValue / totalColor) * 255;
            double normalizedGreen = (greenValue / totalColor) * 255;
            double normalizedBlue = (blueValue / totalColor) * 255;

            final double DOMINANCE_THRESHOLD = 1.2;


            final double BRIGHTNESS_THRESHOLD = 50.0; // Check against total color intensity

            boolean isGreen = (normalizedGreen > normalizedRed * DOMINANCE_THRESHOLD) &&
                    (normalizedGreen > normalizedBlue * DOMINANCE_THRESHOLD) &&
                    (greenValue > BRIGHTNESS_THRESHOLD);

            boolean isPurple = (normalizedBlue > normalizedGreen * DOMINANCE_THRESHOLD) &&
                    (normalizedRed > normalizedGreen * DOMINANCE_THRESHOLD / 2) &&
                    (blueValue > BRIGHTNESS_THRESHOLD);


            if (isGreen) {
                return BallColor.GREEN;
            } else if (isPurple) {
                return BallColor.PURPLE;
            } else {
                return BallColor.PURPLE;
            }
        }

        /**
         * Calculates the required robot rotation in radians to aim a specific shooter at the target.
         *
         * @param servoId The ID of the shooter (1 for left, 2 for center, 3 for right).
         * @param distance The distance to the target in cm (the "adjacent" side of the triangle).
         * @return The angle in radians to turn. Positive for right, negative for left.
         */
        private double getAngleForServo(int servoId, double distance) {
            if (servoId == 2) {
                return 0; // Center shooter is always straight ahead.
            }

            // Using atan(opposite / adjacent) to find the angle.
            double angle = Math.atan(SHOOTER_OFFSET_CM / distance);

            if (servoId == 1) {
                return -angle; // Left shooter requires a negative (counter-clockwise) turn.
            } else { // servoId == 3
                return angle;  // Right shooter requires a positive (clockwise) turn.
            }
        }


        private Servo getServo(int id) {
            switch (id) {
                case 1: return transfer1;
                case 2: return transfer2;
                case 3: return transfer3;
                default: return null;
            }
        }

        public class ShootAction implements Action {
            private final int servoId;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean servoMoved = false;

            public ShootAction(int servoId) {
                this.servoId = servoId;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!servoMoved) {
                    Servo currentServo = getServo(servoId);
                    if (currentServo != null) {
                        currentServo.setPosition(POS_ACTIVE);
                    }
                    timer.reset();
                    servoMoved = true;
                }

                if (timer.seconds() >= WAIT_TIME) {
                    Servo currentServo = getServo(servoId);
                    if (currentServo != null) {
                        currentServo.setPosition(POS_INITIAL);
                    }
                    return false; // Action finished
                }
                return true; // Still waiting
            }
        }

        public Action doTransfer(BallColor[] shootingOrder, double distance) {
            return new RobotCentricShootingAction(shootingOrder, distance);
        }

        public Action doTransfer(BallColor[] shootingOrder) {
            return new RobotCentricShootingAction(shootingOrder, null);
        }

        private class RobotCentricShootingAction implements Action {
            private final int[] sequence;
            private final Double distance;private boolean initialized = false;
            private double startHeading = 0;
            private Action currentAction = null;
            private int sequenceIndex = 0;
            // Define a default distance to use for angle calculations if none is provided.
            // This ensures the robot still turns. Adjust this value based on typical shooting distances.
            private static final double DEFAULT_SHOOTING_DISTANCE_CM = 100.0;


            RobotCentricShootingAction(BallColor[] shootingOrder, Double distance) {
                this.distance = distance;
                // 1. Build the shooting sequence
                HashMap<Integer, BallColor> detectedColors = new HashMap<>();
                detectedColors.put(1, getColorOfSensor(1));
                detectedColors.put(2, getColorOfSensor(2));
                detectedColors.put(3, getColorOfSensor(3));

                List<Integer> builtSequence = new ArrayList<>();
                List<Integer> availablePositions = new ArrayList<>(Arrays.asList(1, 2, 3));

                for (BallColor desiredColor : shootingOrder) {
                    Integer foundPosition = null;
                    for (Integer position : availablePositions) {
                        if (detectedColors.get(position) == desiredColor) {
                            foundPosition = position;
                            break;
                        }
                    }
                    if (foundPosition != null) {
                        builtSequence.add(foundPosition);
                        availablePositions.remove(foundPosition);
                    }
                }
                builtSequence.addAll(availablePositions);
                this.sequence = builtSequence.stream().mapToInt(i -> i).toArray();
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // We need the drive object to perform turns. If it's null, we can't proceed with turning logic.
                if (drive == null) {
                    // Fallback to sequential shooting without turning if drive is not available.
                    if (currentAction == null) {
                        List<Action> actions = new ArrayList<>();
                        for (int servoId : sequence) {
                            actions.add(new ShootAction(servoId));
                        }
                        currentAction = new SequentialAction(actions);
                    }
                    return currentAction.run(packet);
                }

                if (!initialized) {
                    drive.localizer.update();
                    startHeading = drive.localizer.getPose().heading.log();
                    initialized = true;
                }

                // If the current action (turn or shoot) is finished, clear it.
                if (currentAction != null && !currentAction.run(packet)) {
                    currentAction = null;
                }

                // If there is no current action, create the next one in the sequence.
                if (currentAction == null) {
                    if (sequenceIndex < sequence.length) {
                        // Get the servo ID for the current shot.
                        int servoId = sequence[sequenceIndex];

                        // Determine the distance to use for angle calculation.
                        double effectiveDistance = (distance != null) ? distance : DEFAULT_SHOOTING_DISTANCE_CM;

                        // Calculate the turn angle required for this specific servo.
                        double turnAngle = getAngleForServo(servoId, effectiveDistance);
                        double targetHeading = startHeading + turnAngle;

                        // Update the robot's current pose.
                        drive.localizer.update();

                        // Create a sequential action to first turn to the target heading, then shoot.
                        currentAction = new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose()).turnTo(targetHeading).build(),
                                new ShootAction(servoId)
                        );
                        sequenceIndex++;
                    } else if (sequenceIndex == sequence.length) {
                        // After all shots are fired, turn the robot back to its initial heading.
                        drive.localizer.update();
                        currentAction = drive.actionBuilder(drive.localizer.getPose()).turnTo(startHeading).build();
                        sequenceIndex++; // Increment to prevent this block from running again.
                    } else {
                        // All actions are complete.
                        return false;
                    }
                }

                // The current action is still running.
                return true;
            }
        }

    }


    public class CamControl {
        public Camera Cam;
        private ObjectInfo goalTag = null;
        private final String Alliance;
        private final MecanumDrive Drive;

        public CamControl(HuskyLensCam cam, MecanumDrive drive, String alliance) {
            Cam = cam;
            if (!alliance.equalsIgnoreCase("red") && !alliance.equalsIgnoreCase("blue")) {
                throw new IllegalArgumentException("Invalid alliance: " + alliance + " use red or blue");
            }
            Alliance = alliance.toLowerCase();
            Drive = drive;
        }
        public CamControl(LimelightCam cam, MecanumDrive drive, String alliance) {
            Cam = cam;
            if (!alliance.equalsIgnoreCase("red") && !alliance.equalsIgnoreCase("blue")) {
                throw new IllegalArgumentException("Invalid alliance: " + alliance + " use red or blue");
            }
            Alliance = alliance.toLowerCase();
            Drive = drive;
        }
        public ObjectInfo getGoalTag() {
            return goalTag;
        }

        public BallColor[] getShootingOrder() {
            List<ObjectInfo> tags = Cam.scanTag();
            if (tags == null || tags.isEmpty()) {
                // Default order if no tags are seen
                return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE};
            }

            ObjectInfo tag = null;
            for (ObjectInfo tagI : tags){
                if (tagI.objectID != 24 && tagI.objectID != 25){
                    tag = tagI;
                    break;
                }
            }
            switch (tag.objectID) {
                case 21:
                    return new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
                case 22:
                    return new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
                case 23:
                    return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
                default:
                    // Default order for any other tag ID
                    return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE};
            }
        }

        public class AutoAlignGoal implements Action {
            private boolean initialized = false;
            private Action trajectoryAction;

            public AutoAlignGoal(ObjectInfo tag) {
                goalTag = tag;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if (goalTag == null) {
                        return false;
                    }

                    Drive.localizer.update();
                    Pose2d currentPose = Drive.localizer.getPose();
                    Pose2d targetPose = Cam.getPoseOf(currentPose, goalTag);
                    packet.put("targetPose: ", targetPose.toString());
                    packet.put("lat dist in",goalTag.lateralDistance*0.3937);
                    packet.put("currentPose: ", currentPose.toString());
                    packet.put("turn deg",goalTag.yaw);





                    trajectoryAction = Drive.actionBuilder(currentPose)
                         .turn(Math.toRadians(goalTag.yaw))
                           // .strafeTo(new Vector2d(currentPose.position.x,currentPose.position.y-(goalTag.lateralDistance*0.3937)))
                            .build();

                    initialized = true;
                }
                if (trajectoryAction != null) {
                    return trajectoryAction.run(packet);
                }
                return false;
            }
        }

        public Action autoAlignGoal(ObjectInfo goalTag) {
            return new AutoAlignGoal(goalTag);
        }
    }

    public class HuskyLensServo {
        private final Servo hlServo;

        public HuskyLensServo(HardwareMap hardwareMap) {
            hlServo = hardwareMap.get(Servo.class, "hlServo");
        }

        public class LookRight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_RIGHT);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                packet.put("hlServo pos", hlServo.getPosition());
                return false;
            }
        }

        public class LookLeft implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_LEFT);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                packet.put("hlServo pos", hlServo.getPosition());
                return false;
            }
        }

        public class LookForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_FORWARD);
                packet.put("hlServo pos", hlServo.getPosition());
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }

        public class LookBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_BACK);
                packet.put("hlServo pos", hlServo.getPosition());
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }

        public Action lookRight() {
            return new LookRight();
        }

        public Action lookLeft() {
            return new LookLeft();
        }

        public Action lookForward() {
            return new LookForward();
        }

        public Action lookBack() {
            return new LookBack();
        }
    }
}
