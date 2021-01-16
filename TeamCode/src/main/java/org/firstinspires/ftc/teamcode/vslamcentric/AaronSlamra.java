package org.firstinspires.ftc.teamcode.vslamcentric;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.spartronics4915.lib.T265Camera;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.OptionalDouble;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;

public class AaronSlamra {
    private DcMotor[] motors;
    private double[] motorSpeeds;
    private double[] angles;
    private T265Camera camera;
    private Translation2d translation2d;
    private BNO055IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    private Orientation gangles() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); }
    private Telemetry telemetry;

    private boolean driving;
    private boolean turning;

    public void setUp(DcMotor[] motors, double[] angles, T265Camera camera, BNO055IMU imu) {
        setUp(motors, angles, camera, imu, null);
    }

    public void setUp(DcMotor[] motors, double[] angles, T265Camera camera, BNO055IMU imu, Telemetry telemetry) {
        if (motors.length != angles.length) {
            throw new RuntimeException("Motor array length and angle array length are not the same! Check your code!");
        }
        this.motors = motors;
        this.motorSpeeds = new double[motors.length];
        this.angles = angles;
        this.camera = camera;
        this.imu = imu;
        this.telemetry = telemetry;
        this.driving = true;
        this.turning = true;
    }

    public void goToPosition(double moveX, double moveY, TeleAuto callback) {
        goToPosition(moveX, moveY, 1, callback);
    }

    public void goToPosition(double moveX, double moveY, double speed, TeleAuto callback) {
        this.turning = false;
        goTo(moveX, moveY, 0, speed, callback);
        this.turning = true;
    }

    public void goToRotation(double theta, TeleAuto callback) { goToRotation(theta, 1, callback); }

    public void goToRotation(double theta, double speed, TeleAuto callback) {
        this.driving = false;
        goTo(0, 0, theta, speed, callback);
        this.driving = true;
    }

    public void goTo(double moveX, double moveY, double theta, double speed, TeleAuto callback) {
        // Create persistent variables
        boolean xComplete;
        boolean yComplete;
        boolean turnComplete;

        // Wrap theta to localTheta
        double localTheta = wrap(theta);
        while (callback.opModeIsActive()) {
            T265Camera.CameraUpdate up = camera.getLastReceivedCameraUpdate();

            this.translation2d = up.pose.getTranslation();

            double currentX = this.translation2d.getX() / 0.0254;
            double currentY = this.translation2d.getY() / 0.0254;
            double currentTheta = gangles().firstAngle;

            double deltaX = this.driving ? moveX - currentX : 0;
            double deltaY = this.driving ? moveY - currentY : 0;
            double deltaTheta = this.turning ? wrap(localTheta - currentTheta) : 0;

            xComplete = abs(deltaX) < 0.2;
            yComplete = abs(deltaY) < 0.2;
            turnComplete = abs(deltaTheta) < 0.2;

            if (xComplete && yComplete && turnComplete) {
                stopWheel();
                break;
            }

            double driveTheta = Math.atan2(yComplete ? 0 : -deltaY, xComplete ? 0 : deltaX);
            driveTheta += gangles().firstAngle;

            double localSpeed = speed;
            if (abs(deltaX) < 5 && abs(deltaY) < 5) {
                localSpeed *= avg(abs(deltaX), abs(deltaY)) / 10;
            }
            localSpeed = clamp(0.2, 1, localSpeed);

            for (int i = 0; i < this.motors.length; i++) {
                double motorSpeed = (Math.sin(this.angles[i] - driveTheta) + deltaTheta) * localSpeed;
                if (motorSpeed < 0.1 && motorSpeed > -0.1) { motorSpeed = 0; } else
                if (motorSpeed < 0.2 && motorSpeed > 0.1) { motorSpeed = 0.2; } else
                if (motorSpeed > -0.2 && motorSpeed < -0.1) { motorSpeed = -0.2; }
                this.motorSpeeds[i] = motorSpeed;
            }

            OptionalDouble optionalSpeed = Arrays.stream(motorSpeeds).max();
            double fastestSpeed = optionalSpeed.isPresent() ? optionalSpeed.getAsDouble() : 0;
            boolean scale = fastestSpeed > 1;
            double scaleFactor = 1 / fastestSpeed;
            for (int i = 0; i < this.motors.length; i++) {
                this.motors[i].setPower(scale ? this.motorSpeeds[i] * scaleFactor : this.motorSpeeds[i]);
            }

            writeTelemetry(deltaX, deltaY, driveTheta);
        }
    }

    private void stopWheel() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > PI) {
            if (newTheta < -PI) {
                newTheta += 2*PI;
            } else {
                newTheta -= 2*PI;
            }
        }
        return newTheta;
    }

    private double avg(double... inputs) {
        double output = 0;
        for (double input : inputs) {
            output += input;
        }
        output /= inputs.length;
        return output;
    }

    private void writeTelemetry(double deltaX, double deltaY, double driveTheta) {
        if (telemetry == null) { return; }
        packet.put("FR Speed", motors[0].getPower());
        packet.put("RR Speed", motors[1].getPower());
        packet.put("RL Speed", motors[2].getPower());
        packet.put("FL Speed", motors[3].getPower());
        packet.put("Delta X", deltaX);
        packet.put("Delta Y", deltaY);
        packet.put("Drive Theta", driveTheta);
        packet.put("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        dashboard.sendTelemetryPacket(packet);
    }
}
