package org.firstinspires.ftc.teamcode.camera;

import static android.os.SystemClock.sleep;
import static java.lang.Math.tan;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.huskylens.ObjectInfo;
import org.threeten.bp.LocalTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

public interface Camera {

    List<ObjectInfo> scanColor();

    List<ObjectInfo> scanTag();

    List<ObjectInfo> scanAll();

    Pose2d getPoseOf(Pose2d currentPose, ObjectInfo object);

}





