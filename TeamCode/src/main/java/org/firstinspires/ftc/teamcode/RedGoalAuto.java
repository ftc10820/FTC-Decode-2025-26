package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;


/** @noinspection unused*/
@Config
@Autonomous(name = "red goal auto", group = "Autonomous")
public class RedGoalAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        /*
        instantiate MecanumDrive at a particular pose.
        TODO: finalize initial pose with strategy team
        */
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // initialize HuskyLens
//        HuskyLensCam cam = new HuskyLensCam(hardwareMap.get(HuskyLens.class, "huskylens"), 310.47, 200, 29.21, 19.5);

        // initialize actions
//        AutomationsActions automations = new AutomationsActions();
//        AutomationsActions.Shooter shooter = automations.new Shooter(hardwareMap);
//        AutomationsActions.Transfer transfer = automations.new Transfer(hardwareMap);
//        AutomationsActions.HuskyLensDriveControl camControl = automations.new HuskyLensDriveControl(cam, drive, "red");


        // waits for start button to be pressed
        waitForStart();

        // run the actions
        Actions.runBlocking(
                        // drives to large launch zone
                        //TODO: finalize shooting area
                        drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(10, 0), Math.toRadians(0))

                                        .build()



        );

/*
              Code from the Paleolithic Era (old code kept here for reference)

                    List<ObjectInfo> objects=cam.scanTag();
                    while (objects.isEmpty()) objects = cam.scanTag();

                    String tagName = objects.get(0).objectName;
*/

    }
}