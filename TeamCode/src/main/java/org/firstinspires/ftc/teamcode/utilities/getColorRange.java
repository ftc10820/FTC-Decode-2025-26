
package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//
@TeleOp(name = "Testing TeleOP")
@Disabled
public class getColorRange extends LinearOpMode {
    public void initialize() {



            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");



        }







    public ColorSensor colorSensor;







    public void runOpMode() throws InterruptedException {
        int iterations = 0;
        initialize();

        for (;;){
            telemetry.addData("iterations #",iterations);
            telemetry.update();
            if (gamepad1.a){
                iterations++;
            }
            if (gamepad1.b){
                iterations--;
            }
            if (gamepad1.x){
                break;}

        }




        waitForStart();
        double rMax = 0;
        double gMax = 0;
        double bMax = 0;
        double rMin = 999999;
        double gMin = 999999;
        double bMin = 999999;


        for (int i = 0; i < iterations; i++) {
            telemetry.addData("iteration",i);
            telemetry.update();
            sleep(1);
            if (colorSensor.red() > rMax) {
                rMax = colorSensor.red();
            }
            if (colorSensor.green() > gMax) {
                gMax = colorSensor.green();
            }
            if (colorSensor.blue() > bMax) {
                bMax = colorSensor.blue();
            }
            if (colorSensor.red() < rMin) {
                rMin = colorSensor.red();
            }
            if (colorSensor.green() < gMin) {
                gMin = colorSensor.green();
            }
            if (colorSensor.blue() < bMin) {
                bMin = colorSensor.blue();
            }

        }
        while (opModeIsActive()){
            telemetry.addData("rMax",rMax);
            telemetry.addData("gMax",gMax);
            telemetry.addData("bMax",bMax);
            telemetry.addData("rMin",rMin);
            telemetry.addData("gMin",gMin);
            telemetry.addData("bMin",bMin);
            telemetry.update();
        }



    }
}



