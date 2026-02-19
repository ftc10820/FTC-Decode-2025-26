package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "get pose", group = "utils")
@Disabled
public class getPose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        hardwareMap.get(DcMotor.class, "leftFront").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardwareMap.get(DcMotor.class, "leftBack").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardwareMap.get(DcMotor.class, "rightFront").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardwareMap.get(DcMotor.class, "rightBack").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while (opModeIsActive()) {
            drive.localizer.update();
            telemetry.addData("pose: ",drive.localizer.getPose());
            telemetry.update();


        }

    }
}