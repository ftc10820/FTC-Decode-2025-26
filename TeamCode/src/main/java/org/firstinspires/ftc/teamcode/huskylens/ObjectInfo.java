package org.firstinspires.ftc.teamcode.huskylens;
import androidx.annotation.NonNull;

import org.threeten.bp.LocalTime;
public class ObjectInfo {
    int x;
    int y;
    String objectName;
    String objectType;
    double distance;
    double yaw;
    double pitch;
    LocalTime timestamp;

    public ObjectInfo(int x, int y,String objectType, String objectName, double distance,double yaw, double pitch, LocalTime timestamp) {
        this.x = x;
        this.y = y;
        this.objectName = objectName;
        this.objectType = objectType;
        this.distance = distance;
        this.yaw = yaw;
        this.pitch = pitch;
        this.timestamp = timestamp;

    }

    @NonNull
    @Override
    public String toString() {
        return "ObjectInfo [x=" + x + ", y=" + y + ", objectType=" + objectType + ", objectName=" + objectName + ", distance=" + distance + ", yaw=" + yaw + ", pitch=" + pitch + ", timestamp=" +timestamp+ "]";
    }
}