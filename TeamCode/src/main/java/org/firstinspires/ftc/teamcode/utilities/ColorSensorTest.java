package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.AutomationsActions;

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
            telemetry.addData("color sensor 1 color (rgba)",colorSensor1.red()+" "+colorSensor1.green()+" "+colorSensor1.blue()+" "+colorSensor1.alpha());
            telemetry.addData("color sensor 2 color (rgba)",colorSensor2.red()+" "+colorSensor2.green()+" "+colorSensor2.blue()+" "+colorSensor2.alpha());
            telemetry.addData("color sensor 3 color (rgba)",colorSensor3.red()+" "+colorSensor3.green()+" "+colorSensor3.blue()+" "+colorSensor3.alpha());
            telemetry.update();
        }
    }
}
