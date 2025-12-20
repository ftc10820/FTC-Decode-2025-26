
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//
@Config
@TeleOp
public class teleop extends LinearOpMode {
    public void initialize() {

        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        transfer1 = hardwareMap.get(Servo.class,"transfer1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        transfer2 = hardwareMap.get(Servo.class,"transfer2");
        intake3 = hardwareMap.get(CRServo.class, "intake3");
        transfer3 = hardwareMap.get(Servo.class,"transfer3");



        //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    void driveMethod() {

        double y = -gamepad1.left_stick_x;
        double x = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = power * cos / max + turn;
        frontRightPower = power * sin / max - turn;
        backLeftPower = power * sin / max + turn;
        backRightPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {

            frontLeftPower /= power + Math.abs(turn);
            frontRightPower /= power + Math.abs(turn);
            backLeftPower /= power + Math.abs(turn);
            backRightPower /= power + Math.abs(turn);

        }


    }


    public CRServo intake1 = null;

    public Servo transfer1 = null;

    public CRServo intake2 = null;

    public Servo transfer2 = null;

    public CRServo intake3 = null;

    public Servo transfer3 = null;

    // drive train motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotorEx flywheel;
    public DcMotor backRight;
    public final double TICKS_PER_REV = 28.0;
    public final double FLYWHEEL_RPM = 3000;
    public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;



    // there are specific ways that the drive power is calculated based on automations
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;

    //private IntegratingGyroscope NavX;
    // crane linear slide and lifter

    // helper variables
    ElapsedTime eTime1 = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();

    ElapsedTime eTeleOp = new ElapsedTime();

    double speedFactor = 0.8;

    public void runOpMode() throws InterruptedException {
        initialize();
        //pivot encoder homing

        double intakePower =0;
        double flywheelPower = 0;
        double transferPosition = 0;
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake3.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setPower(0);
        intake2.setPower(0);
        intake3.setPower(0);
        transfer1.setPosition(transferPosition);
        transfer2.setPosition(transferPosition);
        transfer3.setPosition(transferPosition);


        waitForStart();
        eTeleOp.reset();
        while (opModeIsActive()) {

            driveMethod();
/*
            if (gamepad2.b && gamepad2.x) {
                intakePower = 0;
            }
            // These conditions change the state, which will persist.
            else if (gamepad2.b) {
                intakePower = 1;  // Run forward
            } else if (gamepad2.x) {
                intakePower = -1; // Run backward
            }
            if (gamepad2.a){
                flywheelPower = 0;
            }
            if (gamepad2.y){
                flywheelPower = 1;
            }
 */
            intakePower = 0;
            if (gamepad1.b) {
                intake1.setPower(1);
                intake2.setPower(0);
                intake3.setPower(0);
            } else if (gamepad1.left_trigger>0) {
                intake2.setPower(1);
                intake1.setPower(0);
                intake3.setPower(0);
            } else if (gamepad1.right_trigger>0) {
                intake3.setPower(1);
                intake1.setPower(0);
                intake2.setPower(0);
            }else{
                intake1.setPower(0);
                intake2.setPower(0);
                intake3.setPower(0);
            }


            if (gamepad2.b) {
                double p = transfer1.getPosition();
                if (p == 1.0d || p == 0.0d) {
                    transfer1.setPosition((p == 1.0d) ? 0 : 1);
                }
                sleep(100);
            }
            if (gamepad2.left_trigger > 0) {
                double p = transfer2.getPosition();
                if (p == 1.0d || p == 0.0d) {
                    transfer2.setPosition((p == 1.0d) ? 0 : 1);
                }
                sleep(100);
            }
            if (gamepad2.right_trigger > 0) {
                double p = transfer3.getPosition();
                if (p == 1.0d || p == 0.0d) {
                    transfer3.setPosition((p == 1.0d) ? 0 : 1);
                }
                sleep(100);
            }
//            if (gamepad2.right_bumper){
//
//                if (transferPosition != 0) {
//                    telemetry.addData("debug","position 0");
//                    telemetry.update();
//                    transferPosition = 0;
//                } else {
//                    telemetry.addData("debug","position 1");
//                    telemetry.update();
//                    transferPosition = 1;
//                        }
//            }
//                sleep(500);
            if (gamepad2.y){
                flywheel.setVelocity(FLYWHEEL_TICKS_PER_REV);
            }else if (gamepad2.a){
                flywheel.setVelocity(0);
             }



            frontLeft.setPower(speedFactor*(frontLeftPower));
            frontRight.setPower(speedFactor*(frontRightPower));
            backLeft.setPower(speedFactor*(backLeftPower));
            backRight.setPower(speedFactor*(backRightPower));



        }
    }
}



