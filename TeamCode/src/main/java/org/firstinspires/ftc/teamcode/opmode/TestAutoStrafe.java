package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Test Auto Strafe", group = "utils")
public class TestAutoStrafe extends LinearOpMode {

    @Override
    public void runOpMode() {
        // The pose `(0, 0, 0)` is facing the positive X axis
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Action moveLeft = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 24))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(moveLeft);
    }
}
