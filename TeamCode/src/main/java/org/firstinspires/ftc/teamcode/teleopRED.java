
package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;
import org.firstinspires.ftc.teamcode.camera.huskylens.ObjectInfo;
import org.threeten.bp.LocalTime;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;


//
@TeleOp(name = "red teleop")
public class teleopRED extends LinearOpMode {
    public void initialize() {

        // setting up drive train
        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
            frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
            backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
            backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            useDrivetrain = true;
            telemetry.addData("Debug", "drivetrain detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no drivetrain detected, proceeding without");

        }
        try{
            flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
            useFlywheel = true;
            telemetry.addData("Debug", "flywheel detected, proceeding with");

        }
        catch (Exception e) {
            telemetry.addData("Debug", "no flywheel detected, proceeding without");

        }
        try{
            intake = hardwareMap.get(DcMotor.class, "intake");
            useIntake = true;
            telemetry.addData("Debug", "intake detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no intake detected, proceeding without");

        }
        try{
            transfer = hardwareMap.get(Servo.class,"transfer");
            transfer2 = hardwareMap.get(Servo.class,"transfer2");
            transfer3 = hardwareMap.get(Servo.class,"transfer3");
            useTransfer = true;
            telemetry.addData("Debug", "transfer detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no transfer detected, proceeding without");

        }
        try {
            colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
            colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");

            useColorSensor = true;
            telemetry.addData("Debug", "color sensor detected, proceeding with. 1: "+colorSensor1.getI2cAddress()+" 2: "+colorSensor2.getI2cAddress()+" 3: "+colorSensor3.getI2cAddress());


        } catch (Exception e) {
            telemetry.addData("Debug", "no color sensor detected, proceeding without");

        }
        try{
            AutomationsActions actions = new AutomationsActions();
            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
            camControl =  actions.new CamControl(new LimelightCam(hardwareMap.get(Limelight3A.class, "limelight"),316.9,  41.91, 9),drive,"red");
            transferControl = actions.new Transfer(hardwareMap, drive);
            shooterControl = actions.new Shooter(hardwareMap);
            hlservo = actions.new HuskyLensServo(hardwareMap);
            LED = hardwareMap.get(Servo.class, "LED1");

            isUseCam = true;
            telemetry.addData("Debug", "cam detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no cam detected, proceeding without");

        }
        telemetry.update();

        if (useFlywheel){
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (useDrivetrain){
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }   }


    void driveMethod() {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }


    }


    public DcMotor intake = null;

    boolean useIntake = false;
    boolean useFlywheel = false;
    boolean useTransfer = false;
    boolean useDrivetrain = false;
    boolean useColorSensor = false;
    public Servo transfer = null;
    public Servo transfer2 = null;
    public Servo transfer3 = null;

    // drive train motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotorEx flywheel;
    public DcMotor backRight;
    MecanumDrive drive;
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;
    public ColorSensor colorSensor3;
    public AutomationsActions.CamControl camControl;
    public AutomationsActions.Transfer transferControl;
    public AutomationsActions.Shooter shooterControl;
    public AutomationsActions.HuskyLensServo hlservo;
    public Servo LED;
    public final double TICKS_PER_REV = 28;
    public final double FLYWHEEL_RPM = 2700;
    public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;
    public boolean isUseCam = false;




    // there are specific ways that the drive power is calculated based on automations
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;

    //private IntegratingGyroscope NavX;
    // crane linear slide and lifter

    // helper variables
    ElapsedTime eTime1 = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();

    ElapsedTime eTeleOp = new ElapsedTime();
    AutomationsActions.BallColor[] shootingOrder;

    double speedFactor = 1;

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
