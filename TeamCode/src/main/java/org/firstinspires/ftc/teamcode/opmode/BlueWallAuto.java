package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.huskylens.ObjectInfo;

import java.util.List;
@Disabled
@Config
@Autonomous(name = "BLUE_WALL_AUTO", group = "Autonomous")
public class BlueWallAuto extends LinearOpMode {
    private int greenposition = -1;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(9, -63, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //TODO: find actual params for huskylens
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Camera mounted 25 cm high, tilted downward 15°
        HuskyLensCam cam = new HuskyLensCam(huskyLens, 316.9, 200, 41.91, 20);

        Shooter shooter = new Shooter(hardwareMap);

        // Go to initial  shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();
        Actions.runBlocking(tab1);
        //  Look at obelisk
        List<ObjectInfo> tags = cam.scanTag();
        if (tags != null) {
            for (ObjectInfo o : tags) {
                switch (o.objectID) {
                    case 21:
                    case 22:
                    case 23:
                        greenposition = o.objectID % 20;
                        break;
                }
            }
        }


        // Go back to Starting Position
        tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-63, 9), Math.toRadians(68))
                .build();
        Actions.runBlocking(tab1);

        //  Launch all pattern balls
        shooter.shootAllBalls(greenposition);

        // ***stretch goal***
        // Move to retrieve balls
        tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-24, 48), Math.toRadians(0))
                .build();
        Actions.runBlocking(tab1);
        // TODO: intake
        // Move to Shooting Position
        tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-63, 9), Math.toRadians(68))
                .build();
        Actions.runBlocking(tab1);

        //  Launch all pattern balls
        shooter.shootAllBalls(greenposition);

    }
}
