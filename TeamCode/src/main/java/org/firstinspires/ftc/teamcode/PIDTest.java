package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PIDTest {
    private DcMotor getMotor() {
        return motor;
    }

    private void setMotor(DcMotor motor) {
        this.motor = motor;
    }

    public DcMotor motor;

    /** Custom PID controller implemented for flywheel shooting
     * @param motor the motor to control, assuming the motor's encoder is plugged onto the same channel as the motor
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     */
    public PIDTest(DcMotorEx motor, double p, double i, double d) {
        /*TODO:
           * set PID coefficients
           * make an actual constructor
         */
        setMotor(motor);
    }
}

