package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutomationsActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.camera.limelight.LimelightCam;

public abstract class TeamLinearOpMode extends LinearOpMode {
    public DcMotor intake = null;

    boolean useIntake = false;
    boolean useFlywheel = false;
    boolean useTransfer = false;
    boolean useDrivetrain = false;
    boolean useColorSensor = false;
    public Servo transfer = null;
    public Servo transfer2 = null;
    public Servo transfer3 = null;

    // drive train motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotorEx flywheel;
    public DcMotor backRight;
    MecanumDrive drive;
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;
    public ColorSensor colorSensor3;
    public AutomationsActions.CamControl camControl;
    public AutomationsActions.Transfer transferControl;
    public AutomationsActions.Shooter shooterControl;
    public AutomationsActions.Intake intakeControl;
    public AutomationsActions.HuskyLensServo hlservo;
    public Servo LED;
    public final double TICKS_PER_REV = 28;
    public final double FLYWHEEL_RPM = 2700;
    public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;
    public boolean isUseCam = false;

    // there are specific ways that the drive power is calculated based on automations
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;

    //private IntegratingGyroscope NavX;
    // crane linear slide and lifter

    // helper variables
    ElapsedTime eTime1 = new ElapsedTime();
    ElapsedTime eTime2 = new ElapsedTime();

    ElapsedTime eTeleOp = new ElapsedTime();
    AutomationsActions.BallColor[] shootingOrder;

    double speedFactor = 1;

    public abstract void runOpMode() throws InterruptedException;

    public void initialize(Pose2d initialPose) {

        // setting up drive train
        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
            frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
            backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
            backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            useDrivetrain = true;
            telemetry.addData("Debug", "drivetrain detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no drivetrain detected, proceeding without");

        }
        try{
            flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
            useFlywheel = true;
            telemetry.addData("Debug", "flywheel detected, proceeding with");

        }
        catch (Exception e) {
            telemetry.addData("Debug", "no flywheel detected, proceeding without");

        }
        try{
            intake = hardwareMap.get(DcMotor.class, "intake");
            useIntake = true;
            telemetry.addData("Debug", "intake detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no intake detected, proceeding without");

        }
        try{
            transfer = hardwareMap.get(Servo.class,"transfer");
            transfer2 = hardwareMap.get(Servo.class,"transfer2");
            transfer3 = hardwareMap.get(Servo.class,"transfer3");
            useTransfer = true;
            telemetry.addData("Debug", "transfer detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no transfer detected, proceeding without");

        }
        try {
            colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
            colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
            colorSensor3 = hardwareMap.get(ColorSensor.class, "colorSensor3");

            useColorSensor = true;
            telemetry.addData("Debug", "color sensor detected, proceeding with. 1: "+colorSensor1.getI2cAddress()+" 2: "+colorSensor2.getI2cAddress()+" 3: "+colorSensor3.getI2cAddress());


        } catch (Exception e) {
            telemetry.addData("Debug", "no color sensor detected, proceeding without");

        }
        try{
            AutomationsActions actions = new AutomationsActions();
            if (initialPose == null) { initialPose = new Pose2d(0,0,0); }
            drive = new MecanumDrive(hardwareMap,initialPose);
            camControl =  actions.new CamControl(new LimelightCam(hardwareMap.get(Limelight3A.class, "limelight"),316.9,  41.91, 9),drive,"red");
            transferControl = actions.new Transfer(hardwareMap, drive);
            shooterControl = actions.new Shooter(hardwareMap);
            intakeControl = actions.new Intake(hardwareMap);
            hlservo = actions.new HuskyLensServo(hardwareMap);
            LED = hardwareMap.get(Servo.class, "LED1");

            isUseCam = true;
            telemetry.addData("Debug", "cam detected, proceeding with");

        } catch (Exception e) {
            telemetry.addData("Debug", "no cam detected, proceeding without");

        }
        telemetry.update();

        if (useFlywheel){
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (useDrivetrain){
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


}
