package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.huskylens.HuskyLensCam;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Red Far Launch Zone Autonomous", group = "Autonomous")
public class RedFarLaunchZoneAUTO extends LinearOpMode {
    public void initialize() {

        // Drivetrain motors
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");

        // Flywheel motor
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");

        // Intake motor
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Transfer servos
        transfer1 = hardwareMap.get(Servo.class,"transfer1");
        transfer2 = hardwareMap.get(Servo.class,"transfer2");
        transfer3 = hardwareMap.get(Servo.class,"transfer3");

        // Set motor power behaviors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Transfer servos
    public Servo transfer1 = null;
    public Servo transfer2 = null;
    public Servo transfer3 = null;


    // HuskyLens and automation actions
    HuskyLens huskyLens = null;
    HuskyLensCam cam = null;
    AutomationsActions.HuskyLens camControl = null;
    AutomationsActions automations = null;


    // Drivetrain motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;


    // Intake and Flywheel motors
    public DcMotorEx intake;
    public DcMotorEx flywheel;


    // TODO: Figure out if these public final doubles are for the flywheel
    public final double TICKS_PER_REV = 28.0;
    public final double FLYWHEEL_RPM = 2700;
    public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Set initial position for starting the match (Red Far Launch Zone)
        Pose2d initialPose = new Pose2d(-63,-24,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Set intake and flywheel motor powers to run constant
        intake.setPower(1);
        flywheel.setPower(1);

        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(12,-16),0)
                .build();
        Actions.runBlocking(tab1);

        // TODO: Read obelisk to get the motif

        Action tab2 = drive.actionBuilder(new Pose2d(new Vector2d(12,-16),0))
                .splineTo(new Vector2d(12,-16),45)
                .build();
        Actions.runBlocking(tab2);

        // TODO: Shoot preloads to score points

        Action tab3 = drive.actionBuilder(new Pose2d(new Vector2d(12,-16),45))
                .splineTo(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5)))
                .build();
        Actions.runBlocking(tab3);

        Action tab4 = drive.actionBuilder(new Pose2d(new Vector2d(-3,-45.75),Math.toRadians(Math.atan(47.625/10.5))))
                .splineTo(new Vector2d(12,-16),45)
                .build();
        Actions.runBlocking(tab4);

        // TODO: Make it shoot the balls

        Action tab5 = drive.actionBuilder(new Pose2d(new Vector2d(7.5,1.875),45))
                // TODO: 9 is a placeholder, need to find real x coordinate
                .splineTo(new Vector2d(9,-42.75),0)
                .build();
        Actions.runBlocking(tab5);

        // TODO: Figure our real tangent to replace placeholder (placeholder = 0)
        // TODO: Control whether tabs 6 and 7 could happen

        Action tab6 = drive.actionBuilder(new Pose2d(new Vector2d(18,-42.75),0))
                .splineTo(new Vector2d(12,-16), 45)
                .build();
        Actions.runBlocking(tab6);

        // TODO: Make it shoot the ball

        Action tab7 = drive.actionBuilder(new Pose2d(new Vector2d(12, -16),45))
                .splineTo(new Vector2d(-24,-24), 0)
                .build();
        Actions.runBlocking(tab7);

}}