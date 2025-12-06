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
            private final Servo servo;

            public Transfer(HardwareMap hardwareMap) {
                servo = hardwareMap.get(Servo.class, "transfer");

            }


            public class DoTransfer implements Action {
                private final ElapsedTime timer = new ElapsedTime();
                private TransferState currentState = TransferState.INITIAL;
                private static final double WAIT_TIME = 1.0;




                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    switch (currentState) {
                        case INITIAL:
                            servo.setPosition(0);
                            currentState = TransferState.POS_1;
                            return true;

                        case POS_1:
                            servo.setPosition(1);
                            timer.reset();
                            currentState = TransferState.WAITING;
                            return true;

                        case WAITING:

                            if (timer.seconds() >= WAIT_TIME) {
                                currentState = TransferState.POS_0;
                            }
                            return true;

                        case POS_0:
                            servo.setPosition(0);
                            currentState = TransferState.FINISHED;
                            return false;

                        case FINISHED:
                            return false;
                    }
                    return true;
                }
            }

            public Action doTransfer() {
                return new DoTransfer();
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
                            if (Objects.equals(o.objectName, "apriltag id #" + 0)) {
                                goalTag = o;
                                packet.put("tag ObjectInfo: ", goalTag.toString());
                                break;
                            }
                        }
                    } else {
                        for (ObjectInfo o : Cam.scanTag()) {
                            // TODO: Get real tag id for blue
                            if (Objects.equals(o.objectName, "apriltag id #" + 24)) {
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
