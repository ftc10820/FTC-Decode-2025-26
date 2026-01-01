package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class PIDTest {
    private DcMotor getMotor() {
        return motor;
    }

    private void setMotor(DcMotor motor) {
        this.motor = motor;
    }

    private double getD() {
        return d;
    }

    private void setD(double d) {
        this.d = d;
    }

    private double getI() {
        return i;
    }

    private void setI(double i) {
        this.i = i;
    }

    private double getP() {
        return p;
    }

    private void setP(double p) {
        this.p = p;
    }

    public DcMotor motor;
    private double p;
    private double i;
    private double d;

    private Telemetry getTelemetry() {
        return telemetry;
    }

    private void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private Telemetry telemetry;



    /** Custom PID controller implemented for flywheel shooting
     * @param motor the motor to control, assuming the motor's encoder is plugged onto the same channel as the motor
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     */
    public PIDTest(DcMotorEx motor, double p, double i, double d, Telemetry telemetry) {
        /*TODO:
           * set PID coefficients
           * make an actual constructor
         */
        setMotor(motor);
        setP(p);
        setI(i);
        setD(d);
        setTelemetry(telemetry);
    }

    /**
     * Test PID controller
     * @param targetVelocity the target velocity in ticks per second
     */
    public void test (double targetVelocity){
        // initializing values
        double kP = getP();
        double kI = getI();
        double kD = getD();
        double dT = 20; // time step in milliseconds
        double previousError = 0;
        double integralSum = 0;
        for (;;){
            double currentVelocity; // in t/s
            double comparePosition;

            // time loop
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            timer.startTime();
            comparePosition = getMotor().getCurrentPosition();
            for (;;){
                if (timer.milliseconds() >= dT){
                    break;
                }
            }
            currentVelocity = (getMotor().getCurrentPosition()-comparePosition)*1000; //approximating t/s from t/ms
            getTelemetry().addData("currentVelocity", currentVelocity);
            getTelemetry().addData("targetVelocity", targetVelocity);
            getTelemetry().update();

            double error = targetVelocity - currentVelocity;
            double proptional = kP * error;
            integralSum += error * dT;
            double integral = kI * integralSum;
            double derivative = kD * (error - previousError) / dT;
            previousError = error;
            double power = proptional + integral + derivative;
            getMotor().setPower(power);
          
        }



    }
}

