package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutomationsActions;

import java.util.Arrays;
//TODO: Update code with newer changes
@Config
@Autonomous(name = "Blue Goal Auto", group = "Autonomous")
public class BlueGoalAuto extends TeamLinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        initialize();
        Pose2d initialPose = new Pose2d(53, 53, Math.toRadians(-135));
        AutomationsActions actions = new AutomationsActions();
        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        AutomationsActions.HuskyLensServo hlServo = actions.new HuskyLensServo(hardwareMap);
        AutomationsActions.Transfer transfer = actions.new Transfer(hardwareMap);


        // Go to initial shooting position
        Action tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .build();
        drive.localizer.update();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(new SequentialAction(tab1,hlServo.lookLeft(),new SleepAction(1)));


        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        sleep(500);
        Actions.runBlocking(new SequentialAction(shooter.spinUp(), new SleepAction(2)));
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();


        Actions.runBlocking(new SequentialAction(transfer.doTransfer(shootingOrder)));
        while(opModeIsActive()) {
            sleep(50);
        }
    }
}
