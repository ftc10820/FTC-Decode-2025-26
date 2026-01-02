package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class AutomationsActions {

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    public class Shooter {
        private final DcMotorEx motor;
        public final double TICKS_PER_REV = 28;
        public final double FLYWHEEL_RPM = 2000;
        public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;

        public Shooter(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "flywheel");

        }

        public class SpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setDirection(DcMotorEx.Direction.REVERSE);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocity(FLYWHEEL_TICKS_PER_REV);
                    initialized = true;
                }

                double vel = motor.getVelocity();
                packet.put("shooterVelocity goal",-FLYWHEEL_TICKS_PER_REV);
                packet.put("shooterVelocity", vel);
                return vel < FLYWHEEL_TICKS_PER_REV;
            }
        }

        public Action spinUp() {
            return new SpinUp();
        }

    }

    public class Transfer {
        private final Servo transfer1, transfer2, transfer3;
        private final ColorSensor colorSensor1, colorSensor2, colorSensor3;

        // Corrected values based on user feedback: Initial is 1.0, Active is 0.5
        private static final double POS_INITIAL = 0;
        private static final double POS_ACTIVE = 1;
        private static final double WAIT_TIME = 1.0; // Time between each servo firing

        public Transfer(HardwareMap hardwareMap) {
            transfer1 = hardwareMap.get(Servo.class, "transfer");
            transfer2 = hardwareMap.get(Servo.class, "transfer2");
            transfer3 = hardwareMap.get(Servo.class, "transfer3");
            transfer1.setDirection(Servo.Direction.REVERSE);
            transfer2.setDirection(Servo.Direction.REVERSE);
            transfer3.setDirection(Servo.Direction.REVERSE);
            colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
            colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");

            transfer1.setPosition(POS_INITIAL);
            transfer2.setPosition(POS_INITIAL);
            transfer3.setPosition(POS_INITIAL);
        }

        private BallColor getColorOfSensor(int sensorId) {
            ColorSensor sensor;
            switch (sensorId) {
                case 1: sensor = colorSensor1; break;
                case 2: sensor = colorSensor2; break;
                case 3: sensor = colorSensor3; break;
                default: return BallColor.NONE;
            }

            final double PRESENCE_THRESHOLD = 100;
            if (sensor.alpha() < PRESENCE_THRESHOLD) {
                return BallColor.NONE;
            }

            double greenValue = Math.max(1, sensor.green());
            double redValue = Math.max(1, sensor.red());
            double blueValue = Math.max(1, sensor.blue());

            final double GREEN_RATIO_THRESHOLD = 1.3;
            final double PURPLE_RATIO_THRESHOLD = 1.2;

            boolean isGreen = greenValue / redValue > GREEN_RATIO_THRESHOLD && greenValue / blueValue > GREEN_RATIO_THRESHOLD;
            boolean isPurple = redValue / greenValue > PURPLE_RATIO_THRESHOLD && blueValue / greenValue > PURPLE_RATIO_THRESHOLD;

            if (isGreen) {
                return BallColor.GREEN;
            } else if (isPurple) {
                return BallColor.PURPLE;
            } else {
                return BallColor.PURPLE;
            }
        }

        public class DoTransfer implements Action {
            private final ElapsedTime timer = new ElapsedTime();
            private final int[] sequence;
            private int currentStep = 0;
            private boolean isWaiting = false;

            public DoTransfer(BallColor[] shootingOrder) {
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

                this.sequence = builtSequence.stream().mapToInt(i->i).toArray();
            }

            private Servo getServo(int id) {
                switch (id) {
                    case 1: return transfer1;
                    case 2: return transfer2;
                    case 3: return transfer3;
                    default: return null;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (currentStep >= sequence.length) {
                    return false;
                }


                if (isWaiting) {
                    if (timer.seconds() >= WAIT_TIME) {
                        isWaiting = false;
                        currentStep++;
                    }
                    return true;
                }


                int servoId = sequence[currentStep];
                Servo currentServo = getServo(servoId);

                if (currentServo != null) {
                    currentServo.setPosition(POS_ACTIVE);
                }


                timer.reset();
                isWaiting = true;

                return true;
            }
        }

        public Action doTransfer(BallColor[] shootingOrder) {
            return new DoTransfer(shootingOrder);
        }
    }
    public class HuskyLens {
        private final HuskyLensCam Cam;
        private ObjectInfo goalTag = null;
        private final String Alliance;
        private final MecanumDrive Drive;

        public HuskyLens(HuskyLensCam cam, MecanumDrive drive, String alliance) {
            Cam = cam;
            if (!alliance.equalsIgnoreCase("red") && !alliance.equalsIgnoreCase("blue")){
                throw new IllegalArgumentException("Invalid alliance: " + alliance+ "\nuse red or blue");
            }
            Alliance = alliance.toLowerCase();
            Drive = drive;
        }

        public BallColor[] getShootingOrder() {
            List<ObjectInfo> tags = Cam.scanTag();
            if (tags == null || tags.isEmpty()) {
                // Default order if no tags are seen
                return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE};
            }

            ObjectInfo tag = tags.get(0);
            switch (tag.objectID) {
                case 1:
                    return new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
                case 2:
                    return new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
                case 3:
                    return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
                default:
                    // Default order for any other tag ID
                    return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.PURPLE};
            }
        }
        public class AutoAlignGoal implements Action {
            private boolean initialized = false;
            private Action trajectoryAction;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if (Objects.equals(Alliance, "red")) {
                        for (ObjectInfo o : Cam.scanTag()) {
                            if (Objects.equals(o.objectID, 1)) {
                                goalTag = o;
                                packet.put("tag ObjectInfo: ", goalTag.toString());
                                break;
                            }
                        }
                    } else {
                        for (ObjectInfo o : Cam.scanTag()) {
                            if (Objects.equals(o.objectID, 2)) {
                                goalTag = o;
                                break;
                            }
                        }
                    }

                    if (goalTag != null) {
                        Pose2d targetPose = Cam.getPoseOf(Drive.localizer.getPose(), goalTag);
                        packet.put("targetPose: ", targetPose.toString());
                        packet.put("currentPose: ", Drive.localizer.getPose().toString());
                        trajectoryAction = Drive.actionBuilder(Drive.localizer.getPose())
                                .turnTo(targetPose.heading)
                                .build();
                    }
                    initialized = true;
                }
                if (trajectoryAction != null) {
                    return trajectoryAction.run(packet);
                }
                return false;
            }
        }
        public Action autoAlignGoal() {
            return new AutoAlignGoal();
        }
    }
    public class HuskyLensServo{
        private final Servo hlServo;
        public HuskyLensServo(HardwareMap hardwareMap){
            hlServo = hardwareMap.get(Servo.class,"hlServo");
        }
        public class LookRight implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_RIGHT);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                packet.put("hlServo pos",hlServo.getPosition());
                return false;
            }
        }
        public class LookForward implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_FORWARD);
                packet.put("hlServo pos",hlServo.getPosition());
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }
        public class LookBack implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hlServo.setPosition(DecodeConstants.HLSERVO_LOOK_BACK);
                packet.put("hlServo pos",hlServo.getPosition());
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }
        public Action lookRight() {return new LookRight();}
        public Action lookForward() {return new LookForward();}
        public Action lookBack() {return new LookBack();}
    }
}
