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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.camera.ObjectInfo;

import java.util.Arrays;

@Config
@Autonomous(name = "RED_GOAL_AUTO", group = "Autonomous")
public class RedGoalAuto extends TeamLinearOpMode {
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        initialize();
        Pose2d initialPose = new Pose2d(53, -53, Math.toRadians(135));



        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .build();






        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(tab1,shooterControl.spinUp(300)));
        Actions.runBlocking(new SequentialAction(hlservo.lookRight(),new SleepAction(0.3)));

        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        Actions.runBlocking(hlservo.lookForward());
        ObjectInfo goalTag;
        for (;;){
            try {
                goalTag = camControl.Cam.scanTag().get(0);
                break;
            } catch (Exception e){
                e.printStackTrace();
            }
        }
        Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();

        sleep(300);
        drive.localizer.update();
        Actions.runBlocking(hlservo.lookForward());

        //Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();

        for (;;){
            try {
                goalTag = camControl.Cam.scanTag().get(0);
                break;
            } catch (Exception e){
                e.printStackTrace();
            }
        }

        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(shooterControl.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(300)));
        drive.localizer.update();
        Action tab2 = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(49)
                .turnTo(Math.PI)
                .build();
        Actions.runBlocking(new SequentialAction(tab2, intakeControl.intakeAction(0.9)));
        drive.localizer.update();
        Action tab3 = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(0,new TranslationalVelConstraint(30.0))
                .build();
        Actions.runBlocking(new SequentialAction(tab3, intakeControl.intakeAction(0)));
        drive.localizer.update();
        Action tab4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(24,-24),Math.PI-Math.toRadians(45),new TranslationalVelConstraint(30.0))
                .build();
        Actions.runBlocking(tab4);
        for (;;){
            try {
                goalTag = camControl.Cam.scanTag().get(0);
                break;
            } catch (Exception e){
                e.printStackTrace();
            }
        }
        drive.localizer.update();
        Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();

        sleep(300);
        drive.localizer.update();
        Actions.runBlocking(hlservo.lookForward());

        //Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();

        for (;;){
            try {
                goalTag = camControl.Cam.scanTag().get(0);
                break;
            } catch (Exception e){
                e.printStackTrace();
            }
        }
        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(shooterControl.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(0)));
        drive.localizer.update();
        telemetry.update();

        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
