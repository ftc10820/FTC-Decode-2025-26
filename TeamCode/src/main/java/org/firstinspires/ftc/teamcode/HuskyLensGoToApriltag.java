package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.List;

@Config
@Autonomous(name = "Move to AprilTag (test)", group = "Autonomous")
public class HuskyLensGoToApriltag extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize huskylens
        HuskyLensCam cam = new HuskyLensCam(hardwareMap.get(HuskyLens.class, "huskylens"), 317.98, 200, 29.21, 19.5);
        waitForStart();
        List<ObjectInfo> objects=cam.scanTag();
        while (objects.isEmpty()) objects = cam.scanTag();

        Pose2d tagPos = cam.getPose(drive.localizer.getPose(), objects.get(0));
        Actions.runBlocking( drive.actionBuilder(initialPose)
                .splineTo(tagPos.position, tagPos.heading).build());

    }
}