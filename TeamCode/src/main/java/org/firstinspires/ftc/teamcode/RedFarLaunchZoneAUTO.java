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
        Pose2d initialPose = new Pose2d(-63,-24,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(12,-16),0)
                .build();
        Actions.runBlocking(tab1);

        // TODO: Read obelisk to get the color pattern

        Action tab2 = drive.actionBuilder(new Pose2d(new Vector2d(12,-16),0))
                .splineTo(new Vector2d(12,-16),45)
                .build();
        Actions.runBlocking(tab2);

        // TODO: Shoot preloads to score points

        Action tab3 = drive.actionBuilder(new Pose2d(new Vector2d(12,-16),45))
                .splineTo(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5)))
                .build();
        Actions.runBlocking(tab3);

        Action tab4 = drive.actionBuilder(new Pose2d(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5))))
                .splineTo(new Vector2d(12,-16),45)
                .build();
        Actions.runBlocking(tab4);

        // TODO: Make it shoot the balls

        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(7.5,1.875),45))
                // TODO: 9 is a placeholder, need to find real x coordinate
                .splineTo(new Vector2d(9,-42.75),0)
                .build();
        Actions.runBlocking(tab5);

        // TODO: Figure our real tangent to replace placeholder (placeholder = 0)
        // TODO: Control whether tabs 6 and 7 could happen

        Action tab6 = drive.actionBuilder(new Pose2d(new Vector2d(18,-42.75),0))
                .splineTo(new Vector2d(12,-16), 45)
                .build();
        Actions.runBlocking(tab6);

        // TODO: Make it shoot the ball

        Action tab7 = drive.actionBuilder(new Pose2d(new Vector2d(12, -16),45))
                .splineTo(new Vector2d(-24,-24), 0)
                .build();
        Actions.runBlocking(tab7);

}}