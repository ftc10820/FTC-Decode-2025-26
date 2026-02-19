package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.camera.huskylens.HuskyLens2;

@TeleOp
@Disabled
public class HuskyLens2Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HuskyLens2 lens=hardwareMap.get(HuskyLens2.class,"huskylens2");
        boolean knock = lens.knock();
        while (opModeIsActive()){
            telemetry.addData("knock",knock);
            telemetry.update();
        }
    }

}
