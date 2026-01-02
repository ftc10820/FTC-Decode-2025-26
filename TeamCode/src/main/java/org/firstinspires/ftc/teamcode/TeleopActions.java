
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;


//
@TeleOp(name = "TeleOP with automations")
public class TeleopActions extends LinearOpMode {

    public void initialize() {

        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        intake = hardwareMap.get(CRServo.class, "intake");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        automations = new AutomationsActions();
        transfer = automations.new Transfer(hardwareMap);
        shooter = automations.new Shooter(hardwareMap);
        camControl = automations.new HuskyLens(new HuskyLensCam(hardwareMap.get(HuskyLens.class, "huskylens"), 247.07, 200, 29.21, 27), new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0))), "red");





        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    void driveMethod() {

        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
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


    public CRServo intake = null;

    //public Servo transfer = null;

    // drive train motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx flywheel;
    public DcMotorEx backRight;

    public AutomationsActions automations;
    public AutomationsActions.Shooter shooter;
    public AutomationsActions.Transfer transfer;
    public AutomationsActions.HuskyLens camControl;


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




        waitForStart();
        eTeleOp.reset();
        double intakePower =0;
        double flywheelPower = 0;
        double transferPosition = 0;

        //transfer.setPosition(transferPosition);
        while (opModeIsActive()) {
            telemetry.addData("flywheel velocity",flywheel.getCurrentPosition());
            telemetry.update();
            driveMethod();

            if (gamepad1.a){
                Actions.runBlocking(camControl.autoAlignGoal());
            }

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
//            if (gamepad2.right_bumper){
//                Actions.runBlocking(transfer.doTransfer(1));
//                }

            //transfer.setPosition(transferPosition);
           flywheel.setPower(flywheelPower);
            intake.setPower(intakePower);
            frontLeft.setPower(speedFactor*(frontLeftPower));
            frontRight.setPower(speedFactor*(frontRightPower));
            backLeft.setPower(speedFactor*(backLeftPower));
            backRight.setPower(speedFactor*(backRightPower));



        }
    }
}



