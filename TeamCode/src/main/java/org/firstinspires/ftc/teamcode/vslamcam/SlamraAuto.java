package org.firstinspires.ftc.teamcode.vslamcam;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.spartronics4915.lib.T265Camera;
import android.content.Context;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

public class SlamraAuto {

    // Defines globally used variables
    private DcMotor[] motors;
    private double[] wheelAngles;
    private double r;
    private double theta;
    private double currentAngle;
    private double currentDegree;
    private Telemetry telemetry;
    private BNO055IMU imu;

    private double[] wheelPowers;

    // Instantiates the T265 camera
    private T265Camera slamra;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Function which is called without telemetry, simply refers to the setUp function with telemetry set to null
    public void setUp(DcMotor[] motors, double[] angles, T265Camera slamra, BNO055IMU imu) {
        setUp(motors, angles, slamra, imu, null);
    }

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotor[] motors, double[] wheelAngles, T265Camera slamra, BNO055IMU imu, Telemetry telemetry) {
        if (motors.length != wheelAngles.length) {
            throw new RuntimeException("Motor array and motor angle array are not the same!");
        }
        this.motors = motors;
        this.wheelAngles = wheelAngles;
        this.wheelPowers = new double[motors.length];
        this.imu = imu;
        this.slamra = slamra;
        this.telemetry = telemetry;
    }

    // Function which is used to update the angle of the robot, used by the drive function
    private void getAngle() {
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        currentDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    // Main function, called to go to a target X and Y position, at a set speed and angle
    public void drive(double targetX, double targetY, double speed, double targetAngle, TeleAuto callback) {

        // Defines any local variables
        boolean turnComplete = false;

        while (callback.opModeIsActive()) {

            // Gathers data from the T265 camera, saving it as a translation2d used to get the current X and Y positions
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up.confidence == T265Camera.PoseConfidence.Failed) {
                continue;
            }
            Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            // Saves the robot's current position each time through the loop
            double currentX = pose.getX();
            double currentY = pose.getY();

            // Calculates the current difference between the target and current positions (the distance between them)
            double diffX = targetX - currentX;
            double diffY = targetY - currentY;
            double diffAvg = (abs(diffX) + abs(diffY)) / 2;

            // Calculates the current difference between the target and current rotations, and converting it to radians
            double diffTurn = targetAngle - currentDegree;
            double diffTurnRad = diffTurn * Math.PI/180;

            // Determines if the robot had reached the target angle
            if (abs(diffTurn) < 0.5) {
                turnComplete = true;
            }

            // Stops robot and ends the loop if the target positions and angle had been completed
            if (abs(diffX) < 0.5 && abs(diffY) < 0.5 && turnComplete) {
                halt();
                break;
            }

            // Calculates the theta, which represents the angle that the robot must drive at in radians
            theta = Math.atan2(diffX, diffY);

            // Calculates the theta, based on how much the robot has turned since the drive had been started
            getAngle();
            double newTheta = theta + currentAngle;

            // Applies a drop in speed as the robot reaches the target position, which increases accuracy
            double newSpeed = speed;
            if (abs(diffX) < 5 && abs(diffY) < 5) {
                newSpeed *= diffAvg / 10;
            }

            // Sets the motor power for each motor based on the theta, speed, and how much the robot must turn.
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(-(Math.sin(wheelAngles[i] - newTheta) * newSpeed + diffTurnRad));
                wheelPowers[i] = motors[i].getPower();
            }

            // Updates all telemetries
            telemetryUpdate(currentX, currentY, diffX, diffY, diffTurn, newSpeed);

            System.out.println("X Location: " + currentX + "\n Y Location: " + currentY + "\n Distance to targetX: " + diffX + "\n Distance to targetY: " + diffY + "\n Current Angle: " + currentDegree + "\n Motor Speeds: " + wheelPowers[0] + " " + wheelPowers[1] + " " + wheelPowers[2] + " " + wheelPowers[3]);
            System.out.println("");

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            final int robotRadius = 9;

            field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
            double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            packet.put("diffX", diffX);
            packet.put("diffY", diffY);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    // Function which stops all motors, used before exiting the drive loop
    private void halt() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    // Function which updates all telemetry
    private void telemetryUpdate(double currentX, double currentY, double diffX, double diffY, double diffTurn, double newSpeed) {
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Distance to X", diffX);
        telemetry.addData("Distance to Y", diffY);
        telemetry.addData("Current Angle", currentDegree);
        telemetry.addData("Distance to full turn", diffTurn);
        telemetry.addData("Current Speed", newSpeed);
        for (int i = 0; i < wheelPowers.length; i++) {
            telemetry.addData("Motor " + i + " Power", wheelPowers[i]);
        }
        telemetry.update();
    }

    // Implements variables which are useful for operating an autonomous, such as opModeIsActive and stopIsRequested
    interface TeleAuto {
        boolean opModeIsActive();
    }
}