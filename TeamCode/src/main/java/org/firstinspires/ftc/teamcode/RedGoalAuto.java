package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.List;

@Config
@Autonomous(name = "Move to AprilTag (test)", group = "Autonomous")
public class RedGoalAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize huskylens
        HuskyLensCam cam = new HuskyLensCam(hardwareMap.get(HuskyLens.class, "huskylens"), 310.47, 200, 29.21, 19.5);
        waitForStart();
        List<ObjectInfo> objects=cam.scanTag();
        while (objects.isEmpty()) objects = cam.scanTag();

        String tagName = objects.get(0).objectName;

    }
}