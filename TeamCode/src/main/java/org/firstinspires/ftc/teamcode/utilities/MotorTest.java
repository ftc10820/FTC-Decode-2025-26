package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "Motor Test")
@Disabled
public class MotorTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            hardwareMap.get(DcMotorEx.class, "test").setPower(1);
        }
}
}
