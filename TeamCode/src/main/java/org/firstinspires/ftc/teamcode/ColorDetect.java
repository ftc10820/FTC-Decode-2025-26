
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//to be removed
@TeleOp(name = "color detect")
public class ColorDetect extends LinearOpMode {
    public void initialize() {



        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");



    }







    public ColorSensor colorSensor;
    public ColorIdentifier colorIdentifier;








    public void runOpMode() throws InterruptedException {
        int iterations = 550;
        initialize();

        for (; ; ) {


            telemetry.addData("info", "starting green");
            telemetry.update();
            waitForStart();
//            double rMaxg = 0;
//            double gMaxg = 0;
//            double bMaxg = 0;
//            double rMing = 999999;
//            double gMing = 999999;
//            double bMing = 999999;
//
//
//            for (int i = 0; i < iterations; i++) {
//                telemetry.addData("green iteration", i);
//                telemetry.update();
//                sleep(1);
//                if (colorSensor.red() > rMaxg) {
//                    rMaxg = colorSensor.red();
//                }
//                if (colorSensor.green() > gMaxg) {
//                    gMaxg = colorSensor.green();
//                }
//                if (colorSensor.blue() > bMaxg) {
//                    bMaxg = colorSensor.blue();
//                }
//                if (colorSensor.red() < rMing) {
//                    rMing = colorSensor.red();
//                }
//                if (colorSensor.green() < gMing) {
//                    gMing = colorSensor.green();
//                }
//                if (colorSensor.blue() < bMing) {
//                    bMing = colorSensor.blue();
//                }
//
//            }
//            telemetry.addData("info", "press x to start purple");
//            telemetry.update();
//            for (; ; ) if (gamepad1.x) break;
//            double rMaxp = 0;
//            double gMaxp = 0;
//            double bMaxp = 0;
//            double rMinp = 999999;
//            double gMinp = 999999;
//            double bMinp = 999999;
//
//
//            for (int i = 0; i < iterations; i++) {
//                telemetry.addData("purple iteration", i);
//                telemetry.update();
//                sleep(1);
//                if (colorSensor.red() > rMaxp) {
//                    rMaxp = colorSensor.red();
//                }
//                if (colorSensor.green() > gMaxp) {
//                    gMaxp = colorSensor.green();
//                }
//                if (colorSensor.blue() > bMaxp) {
//                    bMaxp = colorSensor.blue();
//                }
//                if (colorSensor.red() < rMinp) {
//                    rMinp = colorSensor.red();
//                }
//                if (colorSensor.green() < gMinp) {
//                    gMinp = colorSensor.green();
//                }
//                if (colorSensor.blue() < bMinp) {
//                    bMinp = colorSensor.blue();
//                }
//
//            }
//            colorIdentifier = new ColorIdentifier(colorSensor, rMinp, rMaxp, gMinp, gMaxp, bMinp, bMaxp, rMing, rMaxg, gMing, gMaxg, bMing, bMaxg);
//            telemetry.addData("info", "press x to start detecting");
//            telemetry.update();
//            for (; ; ) if (gamepad1.x) break;
            while (opModeIsActive()) {
                double r = colorSensor.red();
                double g = colorSensor.green();
                double b = colorSensor.blue();
                telemetry.addData("color", "r "+ r + " g " + g + " b " + b);
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                telemetry.update();
            }


        }
    }
}



