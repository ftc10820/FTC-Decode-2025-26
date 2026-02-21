
package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;
import org.firstinspires.ftc.teamcode.camera.ObjectInfo;
import org.threeten.bp.LocalTime;

import java.util.Arrays;
import java.util.List;


//
@TeleOp(name = "red teleop")
public class teleopRED extends TeamTeleopMode {
    public void runOpMode() throws InterruptedException {
        initialize();

        //pivot encoder homing
       Actions.runBlocking(hlservo.lookForward());

        waitForStart();
        eTeleOp.reset();
        double intakePower =0;
        double flywheelPower = 0;

        while (opModeIsActive()) {
            if (isUseCam) {
                drive.localizer.update();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.log()));
            }
            if (useColorSensor){
            telemetry.addData("color sensor 1 color (rgb)",colorSensor1.red()+" "+colorSensor1.green()+" "+colorSensor1.blue());
            telemetry.addData("color sensor 2 color (rgb)",colorSensor2.red()+" "+colorSensor2.green()+" "+colorSensor2.blue());
            telemetry.addData("color sensor 3 color (rgb)",colorSensor3.red()+" "+colorSensor3.green()+" "+colorSensor3.blue());
            if (colorSensor1.green()>colorSensor1.blue() && colorSensor1.green()>colorSensor1.red()&&colorSensor1.green()>65){
                telemetry.addData("color sensor 1","green");

            }else{
                telemetry.addData("color sensor 1","purple");
            }
            if (colorSensor2.green()>colorSensor2.blue() && colorSensor2.green()>colorSensor2.red()&&colorSensor2.green()>65){
                telemetry.addData("color sensor 2","green");

            }else{
                telemetry.addData("color sensor 2","purple");
            }
            if (colorSensor3.green()>colorSensor3.blue() && colorSensor3.green()>colorSensor3.red()&&colorSensor3.green()>65){
                telemetry.addData("color sensor 3","green");

            }else{
                telemetry.addData("color sensor 3","purple");
            }}



            driveMethod();
            LED.setPosition(0);


            // These conditions change the state, which will persist.
            if (gamepad1.x) {
                intakePower = -0.8;  // Run forward
            } else if (gamepad1.b) {
                intakePower = 0.8; // Run backward
            } else if (gamepad1.y){
                intakePower = 0;
            }

            if (gamepad1.left_bumper) {
                if (isUseCam) {
                    Actions.runBlocking(hlservo.lookForward());
                    ObjectInfo goalTag;
                    for (;;){
                        try {
                            goalTag = camControl.Cam.scanTag().get(0);
                            break;
                        } catch (Exception e){

                        }
                    }
                    Actions.runBlocking(camControl.autoAlignGoal(goalTag));
                }
            }


            if (gamepad2.a){
                flywheelPower = 0;
            }
            if (gamepad2.y){
                flywheelPower = 1;
            }
            if (gamepad2.right_bumper){
                try{
                    Actions.runBlocking(hlservo.lookForward());
                    ObjectInfo goalTag;
                    for (;;){
                        try {
                            goalTag = camControl.Cam.scanTag().get(0);
                            break;
                        } catch (Exception e){

                        }
                    }

                Actions.runBlocking(new SequentialAction(camControl.autoAlignGoal(goalTag), shooterControl.spinUp(goalTag),transferControl.doTransfer(shootingOrder,goalTag.distance)));
                telemetry.addData("debug","transfering in order: "+ Arrays.toString(shootingOrder));
                telemetry.update();}
                catch (Exception e){
                    telemetry.addData("debug","no tag detected");
                }
            }
            if (gamepad2.dpad_left){
                transfer.setPosition(0);
            }
            if (gamepad2.dpad_up){
                transfer2.setPosition(0);
            }
            if (gamepad2.dpad_right){
                transfer3.setPosition(0);
            }
            if (gamepad2.dpad_down){
                transfer.setPosition(0.5);
                transfer2.setPosition(0.5);
                transfer3.setPosition(0.5);
            }



            if (useFlywheel){
                if (gamepad2.y){

                        List<ObjectInfo> tags = camControl.Cam.scanTag();
                        final ObjectInfo[] goalTag = new ObjectInfo[1];
                        tags.forEach(tag -> {
                            if (tag.objectID == 24){
                                goalTag[0] = tag;
                            }else {
                                goalTag[0] = null;
                            }


                        });
                        if (goalTag[0] != null){
                        telemetry.addData("cam", goalTag[0].toString());
                        double targetRPM = shooterControl.getRPMFromDistance(goalTag[0].distance, 114.3);
                        telemetry.addData("shooter target rpm", targetRPM);
                        telemetry.update();
                        Actions.runBlocking(shooterControl.spinUp(targetRPM));
                        LED.setPosition(1);
                    } else{
                        telemetry.addData("cam","no tag detected");

                    }
                }
                if (gamepad2.a){
                    flywheel.setPower(0);
                }
                if (gamepad2.b){
                    List<ObjectInfo> tags = camControl.Cam.scanTag();
                    final ObjectInfo[] goalTag = new ObjectInfo[1];
                    tags.forEach(tag -> {
                        if (tag.objectID == 24){
                            goalTag[0] = tag;
                        }else {
                            goalTag[0] = null;
                        }


                    });
                    if (goalTag[0] != null){
                        LED.setPosition(1);
                    }
                }
                telemetry.addData("flywheel",flywheel.getVelocity()+", timestamp : "+ LocalTime.now());

            }
            if (useIntake){
                intake.setPower(intakePower);
            }
            if(isUseCam){
                if (gamepad2.left_trigger>0){
                    try{
                    telemetry.addData("cam", Arrays.toString(camControl.getShootingOrder()));
                    } catch (Exception e){
                        telemetry.addData("cam","no tag detected");

                    }
                }
                if (gamepad1.a){
                    shootingOrder = camControl.getShootingOrder();
                    telemetry.addData("shooting order", Arrays.toString(shootingOrder));
                    telemetry.update();
                }
            }
            if (gamepad2.x){
                transfer.setPosition(.13);
                transfer2.setPosition(.13);
                transfer3.setPosition(.13);
                flywheel.setPower(-0.15);
            }
            if (useDrivetrain) {
                frontLeft.setPower(speedFactor * (frontLeftPower));
                frontRight.setPower(speedFactor * (frontRightPower));
                backLeft.setPower(speedFactor * (backLeftPower));
                backRight.setPower(speedFactor * (backRightPower));
            }
            telemetry.update();

        }
    }
}
