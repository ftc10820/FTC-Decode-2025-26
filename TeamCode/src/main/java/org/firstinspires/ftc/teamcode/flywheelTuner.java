package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.camera.huskylens.HuskyLensCam;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;

@TeleOp(name = "CamControl Tuning", group = "Tuning")
public class flywheelTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutomationsActions actions = new AutomationsActions();
        AutomationsActions.Shooter shooter = actions.new Shooter(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        AutomationsActions.CamControl camControl = actions.new CamControl(new LimelightCam(hardwareMap.get(Limelight3A.class, "limelight"),316.9,  8, -9),drive,"red");



        telemetry.addLine("Auto Tune");
        telemetry.addLine("Place robot facing at target with known distance and RPM.");
        telemetry.update();

        waitForStart();
        //input
       double tunedEfficiency =  shooter.getEfficiencyFactor(2700,200.66,114.3);
        telemetry.addData("Efficiency", tunedEfficiency);
        telemetry.update();
        while (opModeIsActive()) {}
        telemetry.update();


    }
}
