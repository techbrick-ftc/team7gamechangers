package org.firstinspires.ftc.teamcode.vslamcam;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.RotatedRect;

import static java.lang.Math.abs;

public class SimpleSlamra {

    // Defines globally used variables
    private DcMotor[] motors;
    private double currentX;
    private double currentY;
    private Rotation2d rotation;
    private double currentRadian;
    private double currentDegree;
    private T265Camera.PoseConfidence confidence;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private double startingRadian;
    private double startingDegree;
    private double startingX;
    private double startingY;

    private double[] wheelPowers;

    // Instantiates the T265 camera
    private T265Camera slamra;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Function which is called without telemetry, simply refers to the setUp function with telemetry set to null
    public void setUp(DcMotor[] motors, T265Camera slamra, BNO055IMU imu) {
        setUp(motors, slamra, imu, null);
    }

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotor[] motors, T265Camera slamra, BNO055IMU imu, Telemetry telemetry) {
        this.motors = motors;
        this.wheelPowers = new double[motors.length];
        this.imu = imu;
        this.slamra = slamra;
        this.telemetry = telemetry;
        startingDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        startingRadian = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);

        startingX = -pose.getY();
        startingY = pose.getX();

    }

    // Function which is used to update the angle of the robot, used by the drive function
    private void getAngle() {
        currentRadian = wrapRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - startingRadian);
        currentDegree = wrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startingDegree);
    }

    // Main function, called to go to a target X and Y position, at a set speed and angle
    public void drive(double targetX, double targetY, double targetDegree, double speed, TeleAuto callback) {

        double flPower, frPower, rlPower, rrPower;

        while (callback.opModeIsActive()) {
            System.out.println("Start of Loop");

            // Updates angle variables
            getAngle();

            // Updates position variables
            if (!getPosition()) continue;

            // Calculates the current difference between the target and current positions (the distance between them)
            double diffX = targetX - currentX;
            double diffY = targetY - currentY;
            double diffAngle = targetDegree - currentDegree;

            double diffAvg = (abs(diffX) + abs(diffY) + abs(diffAngle)) / 2;

            // Stops robot and ends the loop if the target positions and angle had been completed
            if (abs(diffX) < 1 && abs(diffY) < 1 && abs(diffAngle) < 4) {
                System.out.println("Breaking out of Loop");
                halt();
                break;
            }

            double rotatedX = diffX * Math.cos(-currentRadian) - diffY * Math.sin(-currentRadian);
            double rotatedY = diffY * Math.cos(currentRadian) - diffX * Math.sin(currentRadian);

            flPower = rotatedY + rotatedX - diffAngle;
            rlPower = rotatedY - rotatedX - diffAngle;
            frPower = rotatedY - rotatedX + diffAngle;
            rrPower = rotatedY + rotatedX + diffAngle;

            double max = Math.max(Math.abs(flPower), Math.abs(rlPower));
            max = Math.max(Math.abs(frPower), max);
            max = Math.max(Math.abs(rrPower), max);

            flPower /= max;
            frPower /= max;
            rlPower /= max;
            rrPower /= max;

            double newSpeed = speed;
            if (abs(diffX) < 10 && abs(diffY) < 10 && abs(diffAngle) < 10) {
                newSpeed *= clamp(6, 30, diffAvg) / 30;
            }

            motors[0].setPower(speedCap(flPower * newSpeed));
            motors[1].setPower(speedCap(frPower * newSpeed));
            motors[2].setPower(speedCap(rrPower * newSpeed));
            motors[3].setPower(speedCap(rlPower * newSpeed));

            // Updates all telemetries
            telemetryUpdate(currentX, currentY, diffX, diffY, newSpeed);
            dashUpdate(currentX, currentY, diffX, diffY, newSpeed);

            TelemetryPacket robotPosition = new TelemetryPacket();
            Canvas field = robotPosition.fieldOverlay();
            final int robotRadius = 9;

            field.strokeCircle(-currentY, currentX, robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = -currentY + arrowX  / 2, y1 = currentX + arrowY / 2;
            double x2 = -currentY + arrowX, y2 = currentX + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            dashboard.sendTelemetryPacket(robotPosition);

            System.out.println("Current X: " + currentX + "\nCurrent Y: " + currentY + "\nDiff X: " + diffX + "\nDiff Y: " + diffY + "\nCurrent Radian: " + currentRadian + "\nCurrent Degree: " + currentDegree + "\nConfidence: " + confidence + "\nNew Speed: " + newSpeed + "\nDiff Avg: " + diffAvg + "\nMotor 1 Power: " + motors[0].getPower() + "\nMotor 2 Power: " + motors[1].getPower() + "\nMotor 3 Power: " + motors[2].getPower() + "\nMotor 4 Power: " + motors[3].getPower() + "\nflPower: " + flPower + "\nfrPower: " + frPower + "\nrlPower: " + rlPower + "\nrrPower" + rrPower);
            System.out.println("-");
        }
    }

    // Function which stops all motors, used before exiting the drive loop
    private void halt() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private boolean getPosition() {
        // Gathers data from the T265 camera, saving it as a translation2d used to get the current X and Y positions
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up.confidence == T265Camera.PoseConfidence.Failed) {
            System.out.println("Skipping loop");
            return false;
        }
        Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);

        // Saves the robot's current position
        currentX = -pose.getY() - startingX;
        currentY = pose.getX() - startingY;
        rotation = up.pose.getRotation();
        confidence = up.confidence;
        return true;
    }

    private double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
    }

    private double speedCap(double motorSpeed) {
        if (motorSpeed < 0.05 && motorSpeed > -0.05) { motorSpeed = 0; } else
        if (motorSpeed >= 0.05 && motorSpeed < 0.2) { motorSpeed = 0.2; } else
        if (motorSpeed <= -0.05 && motorSpeed > -0.2) { motorSpeed = -0.2; }
        return motorSpeed;
    }

    // Function which updates all telemetry
    private void telemetryUpdate(double currentX, double currentY, double diffX, double diffY, double newSpeed) {
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Distance to X", diffX);
        telemetry.addData("Distance to Y", diffY);
        telemetry.addData("Current Angle", currentDegree);
        telemetry.addData("Current Speed", newSpeed);
        for (int i = 0; i < wheelPowers.length; i++) {
            telemetry.addData("Motor " + i + " Power", wheelPowers[i]);
        }
        telemetry.update();
    }

    private void dashUpdate(double currentX, double currentY, double diffX, double diffY, double newSpeed) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentX", currentX);
        packet.put("currentY", currentY);
        packet.put("diffX", diffX);
        packet.put("diffY", diffY);
        packet.put("angle (degrees)", currentDegree);
        packet.put("angle (radians)", currentRadian);
        for (int i = 0; i < wheelPowers.length; i++) {
            packet.put("Motor " + i + " Power", wheelPowers[i]);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > 180) {
            if (newTheta < -180) {
                newTheta += 360;
            } else {
                newTheta -= 360;
            }
        }
        return newTheta;
    }

    private double wrapRadians(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
}

// Implements variables which are useful for operating an autonomous, such as opModeIsActive and stopIsRequested
interface TeleAuto {
    boolean opModeIsActive();
}