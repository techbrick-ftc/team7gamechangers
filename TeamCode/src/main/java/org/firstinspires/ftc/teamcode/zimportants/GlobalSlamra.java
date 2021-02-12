package org.firstinspires.ftc.teamcode.zimportants;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class GlobalSlamra {
    private static T265Camera camera = null;

    public static void startCamera(HardwareMap hardwareMap) {
        if (camera == null) {
            camera = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
            try {
                Thread.sleep(2000);
            } catch (Exception ignored) {}
            camera.start();
        }
    }

    public static T265Camera.CameraUpdate getUpdate() {
        return camera.getLastReceivedCameraUpdate();
    }

    public static void setPose(Pose2d pose) {
        camera.setPose(pose);
    }
}