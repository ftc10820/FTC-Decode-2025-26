
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.threeten.bp.LocalTime;


//
@TeleOp(name = "Testing TeleOP")
public class teleop_2 extends LinearOpMode {
    public void initialize() {

        // setting up drive train
        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
            frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
            backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
            backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
            useDrivetrain = true;
            telemetry.addData("Debug", "drivetrain detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no drivetrain detected, proceeding without");

        }
        try{
            flywheel = hardwareMap.get(DcMotor.class,"flywheel");
            useFlywheel = true;
            telemetry.addData("Debug", "flywheel detected, proceeding with");

        }
        catch (Exception e) {
            telemetry.addData("Debug", "no flywheel detected, proceeding without");

        }
        try{
            intake = hardwareMap.get(CRServo.class, "intake2");
            useIntake = true;
            telemetry.addData("Debug", "intake detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no intake detected, proceeding without");

        }
        try{
            transfer = hardwareMap.get(Servo.class,"transfer2");
            useTransfer = true;
            telemetry.addData("Debug", "transfer detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no transfer detected, proceeding without");

        }
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            useColorSensor = true;
            telemetry.addData("Debug", "color sensor detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no color sensor detected, proceeding without");

        }
        telemetry.update();

        if (useFlywheel){
//            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (useDrivetrain){
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }   }


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


    public CRServo intake = null;

    boolean useIntake = false;
    boolean useFlywheel = false;
    boolean useTransfer = false;
    boolean useDrivetrain = false;
    boolean useColorSensor = false;
    public Servo transfer = null;

    // drive train motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor flywheel;
    public DcMotor backRight;
    public ColorSensor colorSensor;





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



        waitForStart();
        eTeleOp.reset();
        double intakePower =0;
        double flywheelPower = 0;
        double transferPosition = 0;
        if (useTransfer) {
            transfer.setPosition(transferPosition);
        }
        while (opModeIsActive()) {
            telemetry.addData("color (alpha)",colorSensor.alpha());
            telemetry.addData("color (rgb)",colorSensor.red()+" "+colorSensor.green()+" "+colorSensor.blue());
            telemetry.update();



            driveMethod();

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
            if (gamepad2.right_bumper){

                if (transferPosition != 0) {
                    telemetry.addData("debug","position 0");
                    telemetry.update();
                    transferPosition = 0;
                } else {
                    telemetry.addData("debug","position 1");
                    telemetry.update();
                    transferPosition = 1;
                }
                sleep(500);
            }
            if (useTransfer){
                transfer.setPosition(transferPosition);
            }
            if (useFlywheel){
                flywheel.setPower(flywheelPower);
                telemetry.addData("flywheel velocity",flywheel.getCurrentPosition()+", timestamp : "+ LocalTime.now());
                telemetry.update();
            }
            if (useIntake){
                intake.setPower(intakePower);
            }
            if (useDrivetrain) {
                frontLeft.setPower(speedFactor * (frontLeftPower));
                frontRight.setPower(speedFactor * (frontRightPower));
                backLeft.setPower(speedFactor * (backLeftPower));
                backRight.setPower(speedFactor * (backRightPower));
            }


        }
    }
}



