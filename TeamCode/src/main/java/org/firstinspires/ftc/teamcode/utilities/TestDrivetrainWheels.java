package org.firstinspires.ftc.teamcode.utilities;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "red_basket_trajectory_test", group = "Autonomous")
@Disabled
public class TestDrivetrainWheels extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException  {
        // instantiate your MecanumDrive at a particular pose.
        waitForStart();
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0.5);
        sleep(3000);
        leftFront.setPower(0);
        leftBack.setPower(0.5);
        sleep(3000);
        leftBack.setPower(0);
        rightBack.setPower(0.5);
        sleep(3000);
        rightBack.setPower(0);
        rightFront.setPower(0.5);
        sleep(3000);
        rightFront.setPower(0);

    }
}
