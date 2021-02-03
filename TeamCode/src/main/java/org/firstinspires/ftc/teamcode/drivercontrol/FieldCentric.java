package org.firstinspires.ftc.teamcode.drivercontrol;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

public class FieldCentric {
    // Variable setup, all will be explained within code
    private DcMotor[] motors;
    private double[] wheelAngles;
    private double r;
    private double theta;
    private double rotation = 0;
    private double currentAngle;
    private BNO055IMU imu;
    private double offsetAngle = 0;

    private double[] wheelPowers;

    public void setUp(DcMotor[] motors, double[] wheelAngles, BNO055IMU imu) throws Exception {
        // Check if we have angles for every motor, and vice versa
        if (motors.length != wheelAngles.length) {
            throw new java.lang.Exception("Motor and wheelAngle arrays do not have same length.\nCheck your code!!!");
        }

        this.motors = motors;
        this.wheelAngles = wheelAngles;
        this.wheelPowers = new double[motors.length];
        this.imu = imu;

        getAngle();
        this.rotation = currentAngle;

        for (double wheelAngle : wheelAngles) {
            wheelAngle -= currentAngle;
        }
    }

    private void getAngle() {
        currentAngle = wrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offsetAngle);
    }

    /**
     * Run every loop to drive robot using field centricity
     * @param x The input to control robot's x movement
     * @param y The input to control robot's y movement
     * @param turn The input to control robot's turn
     */
    public void Drive(double x, double y, double turn) {
        /*
            Get the current angle
         */
        getAngle();

        /*
            Set r (for polar coords) to the distance of the point (x,y) from (0,0)
            (The speed)
         */
        r = Math.sqrt(x*x+y*y);

        /*
            Set theta (for polar coords) to the theta of the point (x,y) to the x-axis at the origin
            (The direction to go)
         */
        theta = Math.atan2(x, y);

        /*
            Add current angle to account for rotation (since we are getting the theta from a controller
            axis (-1.0 to 1.0) we don't know the angle we are currently at
         */
        double newTheta = theta + currentAngle;

        /*
            Sets rotation to current angle, while driver is intentionally turning the robot. This makes
            it so the robot does not turn from things like poor weight distribution.
        */

        /*if (turn != 0) {
            rotation = currentAngle + turn;
        }

        if (rotation > PI) {
            rotation = PI - .01;
        } else if (rotation > -PI) {
            rotation = -PI + .01;
        }

        double newRotation = rotation - currentAngle;*/

        /*
            Get the angle of the wheel and subtract the newTheta (because newTheta is clockwise and
            math is counter-clockwise)
         */
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(Math.sin(wheelAngles[i] - newTheta) * r + (turn / 1.2));
            wheelPowers[i] = motors[i].getPower();
        }
    }

    public void newOffset() {
        offsetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Math.PI/2;
    }

    private double wrap(double theta) {
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
