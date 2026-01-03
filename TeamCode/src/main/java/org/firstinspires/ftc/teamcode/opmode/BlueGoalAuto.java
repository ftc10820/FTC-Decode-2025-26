package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;

import java.util.Arrays;

@Config
@Autonomous(name = "BLUE_GOAL_AUTO", group = "Autonomous")
public class BlueGoalAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(53, 53, Math.toRadians(-135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutomationsActions actions = new AutomationsActions();
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


        HuskyLensCam cam = new HuskyLensCam(huskyLens, 316.9, 200, 41.91, 20);
        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        AutomationsActions.HuskyLensServo hlServo = actions.new HuskyLensServo(hardwareMap);
        AutomationsActions.HuskyLens camControl = actions.new HuskyLens(cam, drive, "red");
        AutomationsActions.Transfer transfer = actions.new Transfer(hardwareMap);


        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(-30)
                .build();
        drive.localizer.update();
        Action tab2 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(-24,0),0)
                .build();
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(new SequentialAction(tab1,hlServo.lookRight(),new SleepAction(1)));


        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        sleep(500);
        Actions.runBlocking(new SequentialAction(shooter.spinUp(), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();


        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder),tab2));
        sleep(10000);
        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
