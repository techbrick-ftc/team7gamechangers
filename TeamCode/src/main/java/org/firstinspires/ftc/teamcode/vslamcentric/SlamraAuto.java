// This was out first attempt at making a camera driving system. It does not work, and is outdated.

package org.firstinspires.ftc.teamcode.vslamcentric;

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

import static java.lang.Math.PI;
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
    public void drive(double targetX, double targetY, double speed, TeleAuto callback) {
        while (callback.opModeIsActive()) {

            // Gathers data from the T265 camera, saving it as a translation2d used to get the current X and Y positions
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up.confidence == T265Camera.PoseConfidence.Failed) {
                continue;
            }
            Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            // Updates angle variables
            getAngle();

            // Saves the robot's current position each time through the loop
            double currentX = pose.getX();
            double currentY = pose.getY();

            // Calculates the current difference between the target and current positions (the distance between them)
            double diffX = targetX - currentX;
            double diffY = targetY - currentY;

            double diffAvg = (abs(diffX) + abs(diffY)) / 2;

            // Stops robot and ends the loop if the target positions and angle had been completed
            if (abs(diffX) < 0.5 && abs(diffY) < 0.5) {
                halt();
                break;
            }

            // Calculates the theta, which represents the angle that the robot must drive at in radians
            theta = Math.atan2(-diffY, diffX);

            // Calculates the theta, based on how much the robot has turned since the drive had been started
            double newTheta = theta + currentAngle;

            // Applies a drop in speed as the robot reaches the target position, which increases accuracy
            double newSpeed = speed;
            if (abs(diffX) < 5 && abs(diffY) < 5) {
                newSpeed *= diffAvg / 10;
            }

            // Sets the motor power for each motor based on the theta, speed, and how much the robot must turn.
            /*for (int i = 0; i < motors.length; i++) {
                motors[i].setPower((Math.sin(wheelAngles[i] - newTheta)) * newSpeed);
                wheelPowers[i] = motors[i].getPower();
            }*/

            // Updates all telemetries
            telemetryUpdate(currentX, currentY, diffX, diffY, newSpeed);
            dashUpdate(currentX, currentY, diffX, diffY, newSpeed, newTheta, theta);

            System.out.println("X Location: " + currentX + "\n Y Location: " + currentY + "\n Distance to targetX: " + diffX + "\n Distance to targetY: " + diffY + "\n Current Angle: " + currentDegree + "\n Motor Speeds: " + wheelPowers[0] + " " + wheelPowers[1] + " " + wheelPowers[2] + " " + wheelPowers[3]);
            System.out.println("");

            TelemetryPacket robotPosition = new TelemetryPacket();
            Canvas field = robotPosition.fieldOverlay();
            final int robotRadius = 9;

            field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
            double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            dashboard.sendTelemetryPacket(robotPosition);
        }
    }

    public void rotate(double targetAngle, double speed, TeleAuto callback) {
        while (callback.opModeIsActive()) {

            // Updates current angle variables
            getAngle();

            // Calculates the difference between the target angle and current angle
            double diffAngle = wrap(targetAngle - currentDegree);

            // Exits loop when in position
            if (abs(diffAngle) < 5) {
                halt();
                break;
            }

            // Makes the speed lessen as it reaches the target angle.
            /*double newSpeed = speed;
            if (abs(diffAngle) < 10) {
                newSpeed = diffAngle / 20;
            }*/

            // Sets motor speeds
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(clamp(-1, 1, diffAngle / 5) * speed);
                wheelPowers[i] = motors[i].getPower();
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current angle", currentDegree);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    // Function which stops all motors, used before exiting the drive loop
    private void halt() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
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

    private void dashUpdate(double currentX, double currentY, double diffX, double diffY, double newSpeed, double newTheta, double theta) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("currentX", currentX);
        packet.put("currentY", currentY);
        packet.put("diffX", diffX);
        packet.put("diffY", diffY);
        packet.put("angle (degrees)", currentDegree);
        packet.put("angle (radians)", currentAngle);
        packet.put("speed", newSpeed);
        packet.put("theta", theta);
        packet.put("newTheta", newTheta);
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
}

// Implements variables which are useful for operating an autonomous, such as opModeIsActive and stopIsRequested
interface TeleAuto {
    boolean opModeIsActive();
}