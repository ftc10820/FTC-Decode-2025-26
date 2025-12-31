package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.Objects;

public class AutomationsActions {
    private enum TransferState {
        INITIAL,
        POS_1,
        WAITING,
        POS_0,
        FINISHED
    }
    public class Shooter {
        private final DcMotorEx motor;

        public Shooter(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        }

        public class SpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setDirection(DcMotorEx.Direction.REVERSE);
                    motor.setPower(0.8);
                    initialized = true;
                }

                double vel = motor.getVelocity();
                packet.put("shooterVelocity", vel);
                return vel < 10_000.0;
            }
        }

        public Action spinUp() {
            return new SpinUp();
        }

    }



    public class Transfer {
        private final Servo transfer1;
        private final Servo transfer2;
        private final Servo transfer3;

        // Define positions for all servos
        private static final double POS_INITIAL = 0.0;
        private static final double POS_ACTIVE = 1.0;
        private static final double WAIT_TIME = 0.5; // Gap time between servo movements

        public Transfer(HardwareMap hardwareMap) {
            transfer1 = hardwareMap.get(Servo.class, "transfer1");
            transfer2 = hardwareMap.get(Servo.class, "transfer2");
            transfer3 = hardwareMap.get(Servo.class, "transfer3");

            // Set all servos to initial position at initialization
            transfer1.setPosition(POS_INITIAL);
            transfer2.setPosition(POS_INITIAL);
            transfer3.setPosition(POS_INITIAL);
        }


        public class DoTransfer implements Action {
            private final ElapsedTime timer = new ElapsedTime();

            private final int[] sequence;
            private int currentStep = 0;
            private boolean isWaiting = false;

            public DoTransfer(int sequenceId) {
                switch (sequenceId) {
                    case 1:
                        this.sequence = new int[]{1, 2, 3};
                        break;
                    case 2:
                        this.sequence = new int[]{2, 1, 3};
                        break;
                    case 3:
                        this.sequence = new int[]{3, 1, 2};
                        break;
                    default:

                        this.sequence = new int[]{1, 2, 3};
                        break;
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

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (currentStep >= sequence.length * 2) {

                    return false;
                }

                if (isWaiting) {
                    if (timer.seconds() >= WAIT_TIME) {
                        isWaiting = false;
                        currentStep++;
                    }
                    return true;
                }


                int servoIndex = currentStep / 2;
                int servoId = sequence[servoIndex];
                Servo currentServo = getServo(servoId);

                if (currentStep % 2 == 0) {

                    if (currentServo != null) currentServo.setPosition(POS_ACTIVE);
                    timer.reset();
                    isWaiting = true;
                } else {

                    if (currentServo != null) currentServo.setPosition(POS_INITIAL);
                    timer.reset();
                    isWaiting = true;
                }

                return true;
            }
        }

        /**
         * Use this to get an instance of the DoTransfer action.
         * @param sequenceId The order of shooting (1, 2, or 3). ]<br>if 1: 1st servo, 2nd servo, 3rd servo<br>if 2: 2nd servo, 1st servo, 3rd servo<br>if 3: 3rd servo, 1st servo, 2nd servo
         * @return The Action object ready to be run.
         * @implNote <br>to run, use Actions.runBlocking(transfer.doTransfer(<strong>SEQUENCE ID HERE</strong>));
         */
        public Action doTransfer(int sequenceId) {
            return new DoTransfer(sequenceId);
        }
    }
    public class HuskyLensDriveControl {
        private final HuskyLensCam Cam;
        private ObjectInfo goalTag = null;
        private final String Alliance;
        private final MecanumDrive Drive;


        public HuskyLensDriveControl(HuskyLensCam cam, MecanumDrive drive, String alliance) {
            Cam = cam;
            if (!alliance.equalsIgnoreCase("red") && !alliance.equalsIgnoreCase("blue")){
                throw new IllegalArgumentException("Invalid alliance: " + alliance+ "\n use red or blue");
            }
            Alliance = alliance.toLowerCase();
            Drive = drive;
        }

        //TODO: change this name to be more practical
        public class AutoAlignGoal implements Action {
            private boolean initialized = false;
            private Action trajectoryAction;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if (Objects.equals(Alliance, "red")) {
                        for (ObjectInfo o : Cam.scanTag()) {
                            // TODO: Get real tag id for red
                            if (Objects.equals(o.objectID, 1)) {
                                goalTag = o;
                                packet.put("tag ObjectInfo: ", goalTag.toString());
                                break;
                            }
                        }
                    } else {
                        for (ObjectInfo o : Cam.scanTag()) {
                            // TODO: Get real tag id for blue
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
                        Drive.localizer.update();
                        trajectoryAction = Drive.actionBuilder(Drive.localizer.getPose())
                                .strafeToLinearHeading(new Vector2d(Drive.localizer.getPose().position.x-0.1, Drive.localizer.getPose().position.y), targetPose.heading)
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

    }
