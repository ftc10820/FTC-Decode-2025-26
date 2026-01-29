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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.camera.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.camera.huskylens.ObjectInfo;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;

import java.util.Arrays;

@Config
@Autonomous(name = "RED_GOAL_AUTO", group = "Autonomous")
public class RedGoalAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(53, -53, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutomationsActions actions = new AutomationsActions();



        LimelightCam cam = new LimelightCam(hardwareMap.get(Limelight3A.class,"limelight"), 316.9,  41.91, 20);

        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        AutomationsActions.HuskyLensServo hlServo = actions.new HuskyLensServo(hardwareMap);
        AutomationsActions.CamControl camControl = actions.new CamControl(cam, drive, "red");
        AutomationsActions.Transfer transfer = actions.new Transfer(hardwareMap);


        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(30)
                .build();






        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(tab1);
        Actions.runBlocking(new SequentialAction(hlServo.lookRight(),new SleepAction(0.3)));

        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        Actions.runBlocking(hlServo.lookForward());
        ObjectInfo goalTag;
        for (;;){
            try {
                goalTag = cam.scanTag().get(0);
                break;
            } catch (Exception e){

            }
        }
        Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();

        sleep(300);
        drive.localizer.update();
        Actions.runBlocking(hlServo.lookForward());

        //Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();


        Actions.runBlocking(new SequentialAction(shooter.spinUp(), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder,goalTag.distance),shooter.spinUp(0)));
        drive.localizer.update();
//        Action tab2 = drive.actionBuilder(drive.localizer.getPose())
//                .lineToX(10)
//                .turnTo(Math.PI)
//                .build();
//        Actions.runBlocking(tab2);
        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
