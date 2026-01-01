package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RED_GOAL_AUTO", group = "Autonomous")
public class RedGoalAuto extends LinearOpMode {
    private int greenposition = -1;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        //TODO: find the actual heading
        Pose2d initialPose = new Pose2d(53, -53, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutomationsActions actions = new AutomationsActions();
        //HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        //TODO: find actual params for huskylens
        //HuskyLensCam cam = new HuskyLensCam(huskyLens, 200.0, 200, 33.02, 19.5);

        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        AutomationsActions.HuskyLensServo hlServo = actions.new HuskyLensServo(hardwareMap);

        // Go to initial  shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(tab1, hlServo.lookRight()));





        //  Look at obelisk
//        List<ObjectInfo> tags = cam.scanTag();

//        if (tags != null) {
//            for (ObjectInfo o : tags) {
//                switch (o.objectID) {
//                    case 21:
//                    case 22:
//                    case 23:
//                        greenposition = o.objectID % 20;
//                        break;
//                }
//            }
//        }


        // Go to far launch zone
//        tab1 = drive.actionBuilder(initialPose)
//                //TODO: find the actual heading
//                .splineTo(new Vector2d(-60, 0), Math.toRadians(333.4349488))
//                .build();
//        Actions.runBlocking(tab1);

        //  Launch all pattern balls
//        shooter.shootAllBalls(greenposition);

        // ***stretch goal***
        // Move to retrieve balls
//        tab1 = drive.actionBuilder(initialPose)
//                .splineTo(new Vector2d(-24, 48), Math.toRadians(0))
//                .build();
//        Actions.runBlocking(tab1);
//        // TODO: intake
//        // Move to Shooting Position
//        tab1 = drive.actionBuilder(initialPose)
//                .splineTo(new Vector2d(-60, 0), Math.toRadians(333.4349488))
//                .build();
//        Actions.runBlocking(tab1);

        //  Launch all pattern balls
//        shooter.shootAllBalls(greenposition);

    }
}
