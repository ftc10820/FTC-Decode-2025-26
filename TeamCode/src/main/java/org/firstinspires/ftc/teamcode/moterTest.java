package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class moterTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "rightFront");
        waitForStart();


        while (opModeIsActive()){
            if (gamepad2.y) {
                motor.setPower(1);
            } else if (gamepad2.a) {
                motor.setPower(0);
            }
        }

}
}

