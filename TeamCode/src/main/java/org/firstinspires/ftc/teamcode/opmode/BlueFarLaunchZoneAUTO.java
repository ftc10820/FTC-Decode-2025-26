package org.firstinspires.ftc.teamcode.opmode;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutomationsActions;

import java.util.Arrays;
//TODO: Update code with newer changes

@Autonomous(name = "Blue Far Launch Zone Autonomous", group = "Autonomous")
public class BlueFarLaunchZoneAUTO extends TeamLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Set initial position for starting the match (Red Far Launch Zone)
        Pose2d initialPose = new Pose2d(-63,18, 0);


        waitForStart();
        double intakePower = 0;

        // Code to set up and shoot the balls to score points in auto
        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20,20), Math.toRadians(135))
                .build();
        drive.localizer.update();

        // Code to read the motif and get the correct shooting order
        // Should shoot the balls in the correct shooting order
        Actions.runBlocking(new SequentialAction(tab1, hlservo.lookLeft()));
        AutomationsActions.BallColor[] shootingOrder = camControl.getShootingOrder();
        telemetry.addData("Ball Order", Arrays.toString(shootingOrder));
        telemetry.update();

        Actions.runBlocking(new SequentialAction(new ParallelAction(tab1,shooterControl.spinUp())));
        Actions.runBlocking(new SequentialAction(transferControl.doTransfer(shootingOrder)));

        // Code to leave the launch zone and position to intake
        Action tab2 = drive.actionBuilder(new Pose2d(new Vector2d(0,0),Math.toRadians(135)))
                .splineTo(new Vector2d(28,48),Math.toRadians(-180))
                .build();
                // Get intake running
                intakePower = -0.8;  // Run forward
        Actions.runBlocking(tab2);

        if (useIntake) {
            intake.setPower(intakePower);
        }

        // Code to intake three more balls
        Action tab3 = drive.actionBuilder(new Pose2d(new Vector2d(0,48), Math.toRadians(-180)))
                .splineTo(new Vector2d(12,48),Math.toRadians(-180))
                .build();
                // Get Intake to Stop
                intakePower = 0; // Stop running
        Actions.runBlocking(tab3);

        // Code to get into shooting position to launch the balls
        Action tab4 = drive.actionBuilder(new Pose2d(new Vector2d(-12,-48),Math.toRadians(-180)))
                .splineTo(new Vector2d(20,20),Math.toRadians(135))
                .build();
        Actions.runBlocking(tab4);

        // Get it to run the flywheel and shoot the balls following the motif
        Actions.runBlocking(new SequentialAction(new ParallelAction(tab1,shooterControl.spinUp())));
        Actions.runBlocking(new SequentialAction(transfergControl.doTransfer(shootingOrder)));

        // Code to leave the launch zone for extra points at end of auto
        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(20,-20),Math.toRadians(-135)))
                .splineTo(new Vector2d(0,24),Math.toRadians(-180))
                .build();
        Actions.runBlocking(tab5);

        if (useIntake) {
            intake.setPower(intakePower);
        }

        /* Action tab6 = drive.actionBuilder(new Pose2d(new Vector2d(9,-42.75),Math.toRadians(Math.atan(18.75/3))))
                .splineTo(new Vector2d(12,-24), 45)
                .build();
        Actions.runBlocking(tab6);

        // TODO: Make it shoot the ball

        Action tab7 = drive.actionBuilder(new Pose2d(new Vector2d(12, -24),45))
                .splineTo(new Vector2d(-24,-24), 0)
                .build();
        Actions.runBlocking(tab7);
*/
    }}