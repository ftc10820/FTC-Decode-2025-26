package org.firstinspires.ftc.teamcode.huskylens;
import androidx.annotation.NonNull;

import org.threeten.bp.LocalTime;
public class ObjectInfo {
    int x;
    int y;
    public int objectID;
    public String objectType;
    public double distance;
    public double yaw;
    public double pitch;
    public double realHeight; // Calculated real-world height
    public double lateralDistance; // Calculated lateral distance
    LocalTime timestamp;

    public ObjectInfo(int x, int y, String objectType, int objectID, double distance,double lateralDistance,double yaw, double pitch, double realHeight, LocalTime timestamp) {
        this.x = x;
        this.y = y;
        this.objectID = objectID;
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
        return "ObjectInfo [x=" + x + ", y=" + y + ", objectType=" + objectType + ", objectName=" + objectID + ", distance=" + distance + ", yaw=" + yaw + ", pitch=" + pitch + ", realHeight=" + realHeight + ", timestamp=" +timestamp+ "]";
    }
}