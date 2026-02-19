package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Drivetrain TeleOp")
@Disabled
public class DrivetrainTeleOp extends LinearOpMode {

    // drive train motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    boolean useDrivetrain = false;

    // drive power variables
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;
    double speedFactor = 0.8;

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
        telemetry.update();

        if (useDrivetrain) {
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void driveMethod() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
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

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            driveMethod();


                frontLeft.setPower(speedFactor * (frontLeftPower));
                frontRight.setPower(speedFactor * (frontRightPower));
                backLeft.setPower(speedFactor * (backLeftPower));
                backRight.setPower(speedFactor * (backRightPower));

            
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.update();
        }
    }

}
