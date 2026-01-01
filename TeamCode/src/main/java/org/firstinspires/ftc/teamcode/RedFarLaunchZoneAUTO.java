package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.List;
@Autonomous(name = "Red Far Launch Zone Autonomous", group = "Autonomous")
public class RedFarLaunchZoneAUTO extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Replace (0,0,0) with actual coordinates
        Pose2d initialPose = new Pose2d(-63,-24,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(12,-16),0)
                .build();
        // TODO: Make new positions later
        Actions.runBlocking(tab1);
        // TODO: Read obelisk
        Action tab2 = drive.actionBuilder(new Pose2d(new Vector2d(12,-16),0))
                .splineTo(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5)))
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5))))
                // TODO: Check if arrow actually leads to: (new Vector2d(12,-16),0)
                .splineTo(new Vector2d(7.5,1.875),45)
                .build();
        // TODO: Make it shoot the ball
        Action tab4 = drive.actionBuilder(new Pose2d(new Vector2d(7.5,1.875),45))
                // TODO: Figure out real tangent (0 is placeholder)
                .splineTo(new Vector2d(18,-42.75),0)
                .build();
        // TODO: Figure our real tangent to replace placeholder (placeholder = 0)
        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(18,-42.75),0))
                .splineTo(new Vector2d(12,-16), 45)
                .build();

}}