package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

public interface Camera {

    List<ObjectInfo> scanColor();

    List<ObjectInfo> scanTag();

    List<ObjectInfo> scanAll();

    Pose2d getPoseOf(Pose2d currentPose, ObjectInfo object);

}





