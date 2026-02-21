package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.camera.huskylens.HuskyLensCam;

import java.util.Arrays;
//TODO: Update code with newer changes
@Config
@Autonomous(name = "Blue Goal Auto", group = "Autonomous")
public class BlueGoalAuto extends TeamLinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(53, 53, Math.toRadians(-135));
        initialize(initialPose);

        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .build();
        drive.localizer.update();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(new SequentialAction(tab1,hlservo.lookLeft(),new SleepAction(1)));


        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        sleep(500);
        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();


        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder)));
        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
