package org.firstinspires.ftc.teamcode.huskylens;
import androidx.annotation.NonNull;

import org.threeten.bp.LocalTime;
public class ObjectInfo {
    int x;
    int y;
    public String objectName;
    String objectType;
    double distance;
    double yaw;
    double pitch;
    double realHeight; // Calculated real-world height
    double lateralDistance; // Calculated lateral distance
    LocalTime timestamp;

    public ObjectInfo(int x, int y, String objectType, String objectName, double distance,double lateralDistance,double yaw, double pitch, double realHeight, LocalTime timestamp) {
        this.x = x;
        this.y = y;
        this.objectName = objectName;
        this.objectType = objectType;
        this.distance = distance;
        this.lateralDistance = lateralDistance;
        this.yaw = yaw;
        this.pitch = pitch;
        this.realHeight = realHeight;
        this.timestamp = timestamp;

    }

    @NonNull
    @Override
    public String toString() {
        return "ObjectInfo [x=" + x + ", y=" + y + ", objectType=" + objectType + ", objectName=" + objectName + ", distance=" + distance + ", yaw=" + yaw + ", pitch=" + pitch + ", realHeight=" + realHeight + ", timestamp=" +timestamp+ "]";
    }
}