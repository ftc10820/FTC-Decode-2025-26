package org.firstinspires.ftc.teamcode.huskylens;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "HuskyLens OOP Test", group = "Sensor")

public class HuskyLensOOPTest extends LinearOpMode {
    void multiTelemetry(String caption, String value){
        telemetry.addData(caption,value);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put(caption,value);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }

    private final int READ_PERIOD = 1;

    @Override
    public void runOpMode()
    {
        HuskyLensCam camera = new HuskyLensCam(hardwareMap.get(HuskyLens.class, "huskylens"),348,1100);


        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();


        multiTelemetry(">>", "Press start to continue");

        waitForStart();


        while(opModeIsActive()) {
            List<ObjectInfo> objects = camera.scanTag();
            multiTelemetry("HuskyLens>>",objects.toString());

            telemetry.update();
            sleep(3000);
        }
    }
}