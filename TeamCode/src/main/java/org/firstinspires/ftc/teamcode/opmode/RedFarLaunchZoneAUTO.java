package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
import java.util.List;

@Config
@Autonomous(name = "Red Far Launch Zone", group = "Autonomous")
public class RedFarLaunchZoneAUTO extends TeamLinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        initialize();
        Pose2d initialPose = new Pose2d(-63,-18, Math.PI);



        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(-57)
                .build();





        Actions.runBlocking(hlservo.lookForward());
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(new ParallelAction(shooterControl.spinUp(1300),tab1));
        telemetry.addLine("went forward");
        telemetry.update();
        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        telemetry.addLine("got shooting order");
        telemetry.update();
        List<ObjectInfo> tags;
        ObjectInfo goalTag = null;
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).turnTo(Math.PI-Math.toRadians(20)).build());
        telemetry.addLine("turned");
        telemetry.update();
//        searchForTag:
//        for (;;){
//            try {
//                tags = camControl.Cam.scanTag();
//                for (ObjectInfo tag : tags){
//                if (tag.objectID == 24){
//                    goalTag = tag;
//                    break searchForTag;
//                }}
//            } catch (Exception e){
//                e.printStackTrace();
//            }
//        }
//        Actions.runBlocking(camControl.autoAlignGoal(goalTag));
//        telemetry.addLine("got goal tag");
//        telemetry.update();
//        drive.localizer.update();
//
//        sleep(300);
//        drive.localizer.update();


        //Actions.runBlocking(camControl.autoAlignGoal(goalTag));
        drive.localizer.update();


        goalTag = null;
        searchForTag:
        for (;;){
            try {
                tags = camControl.Cam.scanTag();
                for (ObjectInfo tag : tags){
                    if (tag.objectID == 24){
                        goalTag = tag;
                        break searchForTag;
                    }}
            } catch (Exception e){
                e.printStackTrace();
            }
        }
        telemetry.addLine("got goal tag again");
        telemetry.update();
        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(shooterControl.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
<<<<<<< HEAD
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(1300)));
=======
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(600)));
>>>>>>> 7c6d6b3d1440b391cc04d29684e6bf19d2a4b082
        drive.localizer.update();

        Action preIntake = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Math.toRadians(-90))
                .strafeTo(new Vector2d(-67.2, -40),new TranslationalVelConstraint(200))
                .build();
<<<<<<< HEAD


        Action finalMove = drive.actionBuilder(new Pose2d(-67.2, -40, Math.toRadians(-90)))
                .lineToY(-70,new TranslationalVelConstraint(15.0))
                .build();


        Actions.runBlocking(new SequentialAction(
                preIntake,
                new ParallelAction(
                        finalMove,
                        intakeControl.intakeAction(0.6)
                ),
                new SleepAction(0.5)

        ));

=======
        Actions.runBlocking(new SequentialAction(intakeControl.intakeAction(0.9),tab2,new SleepAction(0.5),intakeControl.intakeAction(0)));
>>>>>>> 7c6d6b3d1440b391cc04d29684e6bf19d2a4b082
        Action tab3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-55,-18),Math.PI-Math.toRadians(20),new TranslationalVelConstraint(100))
                .build();
        Actions.runBlocking(new SequentialAction(tab3,intakeControl.intakeAction(0)));
        goalTag = null;
        searchForTag:
        for (;;){
            try {
                tags = camControl.Cam.scanTag();
                for (ObjectInfo tag : tags){
                    if (tag.objectID == 24){
                        goalTag = tag;
                        break searchForTag;
                    }}
            } catch (Exception e){
                e.printStackTrace();
            }
        }

        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(shooterControl.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
<<<<<<< HEAD
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(1300)));
=======
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(600)));
>>>>>>> 7c6d6b3d1440b391cc04d29684e6bf19d2a4b082
        drive.localizer.update();

        Action tab4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-50,-53),0,new TranslationalVelConstraint(200))
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(-50,-53),0))
                .lineToX(-24,new TranslationalVelConstraint(15.0))
                .build();
<<<<<<< HEAD
        Actions.runBlocking(new SequentialAction(intakeControl.intakeAction(0.6),tab4,tab5));
=======
        Actions.runBlocking(new SequentialAction(intakeControl.intakeAction(0.9),tab4,tab5,intakeControl.intakeAction(0)));
>>>>>>> 7c6d6b3d1440b391cc04d29684e6bf19d2a4b082
        Action tab6 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-55,-18),Math.PI-Math.toRadians(30),new TranslationalVelConstraint(200))
                .build();
        Actions.runBlocking(new SequentialAction(tab6,intakeControl.intakeAction(0)));
        goalTag = null;
        searchForTag:
        for (;;){
            try {
                tags = camControl.Cam.scanTag();
                for (ObjectInfo tag : tags){
                    if (tag.objectID == 24){
                        goalTag = tag;
                        break searchForTag;
                    }}
            } catch (Exception e){
                e.printStackTrace();
            }
        }

        Actions.runBlocking(new SequentialAction(shooterControl.spinUp(shooterControl.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder,goalTag.distance),shooterControl.spinUp(0)));
        drive.localizer.update();


        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
