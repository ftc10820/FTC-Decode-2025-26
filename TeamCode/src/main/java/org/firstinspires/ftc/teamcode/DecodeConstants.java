package org.firstinspires.ftc.teamcode;

public class DecodeConstants {

    // Shooter constants
    public static final double FLYWHEEL_RADIUS_CM = 5.0;
    public static final double LAUNCH_ANGLE_DEGREES = 55.0;
    public static final double SHOOTER_EFFICIENCY = 0.7904;
    public static final double LAUNCH_HEIGHT_CM = 20.0;

    // CamControl Servo constants
    public static final double HLSERVO_LOOK_RIGHT = 0;
    public static final double HLSERVO_LOOK_LEFT = 0.6;
    public static final double HLSERVO_LOOK_FORWARD = 0.3;
    public static final double HLSERVO_LOOK_BACK = 1.1;

    public final double TICKS_PER_REV = 28.0;
    public final double FLYWHEEL_RPM = 2700;
    public final double FLYWHEEL_TICKS_PER_REV = TICKS_PER_REV * FLYWHEEL_RPM / 60.0;
}
