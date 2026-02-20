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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.camera.ObjectInfo;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "Red Far Launch Zone", group = "Autonomous")
public class RedFarLaunchZoneAUTO extends LinearOpMode {

    // TODO: Similar to the OpModes, it would be a good idea to have all initialization/DCMotorExs, Servos, ColorSensors, Camera, etc.
    // in a single class. I strongly recommend creating an abstract class that extends LinearOpMode which will be the parent class
    // of your autonomous OpModes.
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-63,-18, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutomationsActions actions = new AutomationsActions();



        LimelightCam cam = new LimelightCam(hardwareMap.get(Limelight3A.class,"limelight"), 316.9,  41.91, 9);

        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        AutomationsActions.HuskyLensServo hlServo = actions.new HuskyLensServo(hardwareMap);
        AutomationsActions.CamControl camControl = actions.new CamControl(cam, drive, "red");
        AutomationsActions.Transfer transfer = actions.new Transfer(hardwareMap, drive);
        AutomationsActions.Intake intake = actions.new Intake(hardwareMap);



        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(-50.5)
                .build();





        Actions.runBlocking(hlServo.lookForward());
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(tab1);
        telemetry.addLine("went forward");
        telemetry.update();
        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        telemetry.addLine("got shooting order");
        telemetry.update();
        List<ObjectInfo> tags;
        ObjectInfo goalTag = null;
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).turnTo(Math.PI-Math.toRadians(25)).build());
        telemetry.addLine("turned");
        telemetry.update();
//        searchForTag:
//        for (;;){
//            try {
//                tags = cam.scanTag();
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
                tags = cam.scanTag();
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
        Actions.runBlocking(new SequentialAction(shooter.spinUp(shooter.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder,goalTag.distance),shooter.spinUp(300)));
        drive.localizer.update();
        Action tab2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-67.2,-73),Math.toRadians(-90),new TranslationalVelConstraint(15.0))
                .build();
        Actions.runBlocking(new SequentialAction(intake.intakeAction(0.9),tab2,new SleepAction(0.5),intake.intakeAction(0)));
        Action tab3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-55,-18),Math.toRadians(25))
                .build();
        Actions.runBlocking(tab3);
        goalTag = null;
        searchForTag:
        for (;;){
            try {
                tags = cam.scanTag();
                for (ObjectInfo tag : tags){
                    if (tag.objectID == 24){
                        goalTag = tag;
                        break searchForTag;
                    }}
            } catch (Exception e){
                e.printStackTrace();
            }
        }

        Actions.runBlocking(new SequentialAction(shooter.spinUp(shooter.getRPMFromDistance(goalTag.distance,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder,goalTag.distance),shooter.spinUp(300)));
        drive.localizer.update();

        Action tab4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-40,-48),0)
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(-40,-48),0))
                .lineToX(-24,new TranslationalVelConstraint(15.0))
                .build();
        Actions.runBlocking(new SequentialAction(intake.intakeAction(0.9),tab4,tab5,intake.intakeAction(0)));
        Action tab6 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-63,-18),Math.toRadians(25))
                .build();
        Actions.runBlocking(tab6);
        goalTag = null;
        searchForTag:
        for (;;){
            try {
                tags = cam.scanTag();
                for (ObjectInfo tag : tags){
                    if (tag.objectID == 24){
                        goalTag = tag;
                        break searchForTag;
                    }}
            } catch (Exception e){
                e.printStackTrace();
            }
        }

        Actions.runBlocking(new SequentialAction(shooter.spinUp(shooter.getRPMFromDistance(goalTag.distance+45,114.3)), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        drive.localizer.update();
        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder,goalTag.distance+45),shooter.spinUp(0)));
        drive.localizer.update();


        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
