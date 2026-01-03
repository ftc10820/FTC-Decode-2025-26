package org.firstinspires.ftc.teamcode.huskylens;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;

@TeleOp(name = "HuskyLens Tuning", group = "Tuning")
public class HuskyLensTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Camera mounted 25 cm high, tilted downward 15°
        HuskyLensCam cam = new HuskyLensCam(huskyLens, 316.9, 200, 41.91, 20);

        telemetry.addLine("Auto Tune");
        telemetry.addLine("Place target at known height & distance.");
        telemetry.update();

        waitForStart();

        double tunedFocal = cam.autoTuneFocalLength(this, 109.22
                ,73.66, "apriltag", 150);

        telemetry.addLine("✅ Done!");
        telemetry.addData("Focal length", "%.2f px", tunedFocal);
        telemetry.update();

        sleep(5000);
    }
}
