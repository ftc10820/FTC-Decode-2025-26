package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FlywheelCheck extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        Servo tranfer2 = hardwareMap.get(Servo.class,"transfer2");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        final double TICKS_PER_REV = 28;
        final double FLYWHEEL_RPM = 2500;
        double targetVelocity = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;
        flywheel.setVelocity(targetVelocity);
        while (flywheel.getVelocity() < targetVelocity){

        }
        tranfer2.setPosition(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (flywheel.getVelocity() < targetVelocity){

        }
        telemetry.addData("time",timer.seconds());
        while (opModeIsActive()){

            telemetry.update();
        }


    }
}
