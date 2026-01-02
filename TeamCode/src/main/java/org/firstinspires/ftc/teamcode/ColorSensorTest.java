package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutomationsActions actions = new AutomationsActions();
        AutomationsActions.Transfer transfer = actions.new Transfer(hardwareMap);
        ColorSensor colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        ColorSensor colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
        ColorSensor colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("color sensor 1", transfer.getColorOfSensor(1) );
            telemetry.addData("color sensor 2", transfer.getColorOfSensor(2));
            telemetry.addData("color sensor 3", transfer.getColorOfSensor(3));
            telemetry.addData("color sensor 1 color (rgb)",colorSensor1.red()+" "+colorSensor1.green()+" "+colorSensor1.blue());
            telemetry.addData("color sensor 2 color (rgb)",colorSensor2.red()+" "+colorSensor2.green()+" "+colorSensor2.blue());
            telemetry.addData("color sensor 3 color (rgb)",colorSensor3.red()+" "+colorSensor3.green()+" "+colorSensor3.blue());
            telemetry.update();
        }
    }
}
