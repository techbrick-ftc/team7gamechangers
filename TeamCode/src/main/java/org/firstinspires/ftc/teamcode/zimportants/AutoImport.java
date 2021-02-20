// Our parent auto program, which supplies all other auto programs.

package org.firstinspires.ftc.teamcode.zimportants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;

public class AutoImport extends LinearOpMode implements TeleAuto {
    // Defines vars
    protected DcMotor m1 = null;
    protected DcMotor m2 = null;
    protected DcMotor m3 = null;
    protected DcMotor m4 = null;

    protected DcMotor wobbleAxis1 = null;
    protected Servo wobbleAxis2 = null;
    protected DcMotor intake1 = null;
    protected DcMotor intake2 = null;
    protected DcMotorEx shooter = null;
    protected Servo shooterServo = null;
    protected CRServo tapeMeasure = null;
    protected TouchSensor armTouch = null;

    protected SimpleSlamra slauto = new SimpleSlamra();
    protected EasyOpenCVImportable camera = new EasyOpenCVImportable();
    protected BNO055IMU imu;

    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    // vars used in program
    protected int activeGoal;
    protected int startingPoseX;
    protected int startingPoseY;
    protected int cameraX;
    protected int cameraY;

    public AutoImport(int startX, int startY, int camX, int camY) {
        startingPoseX = startX;
        startingPoseY = startY;
        cameraX = camX;
        cameraY = camY;
    }

    public boolean driverAbort() {
        return false;
    }

    public void runOpMode() {
        // configures hardware
        m4 = hardwareMap.get(DcMotor.class, "fl");
        m1 = hardwareMap.get(DcMotor.class, "fr");
        m3 = hardwareMap.get(DcMotor.class, "rl");
        m2 = hardwareMap.get(DcMotor.class, "rr");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        wobbleAxis1 = hardwareMap.get(DcMotor.class, "wobble_axis_1");
        wobbleAxis2 = hardwareMap.get(Servo.class, "wobble_axis_2");
        intake1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake2 = hardwareMap.get(DcMotor.class, "intake_2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterServo = hardwareMap.get(Servo.class, "shooter_servo");
        tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");
        armTouch = hardwareMap.get(TouchSensor.class, "arm_touch");

        wobbleAxis1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // initializes imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(param);

        telemetry.addLine("IMU Done");
        telemetry.update();

        // initializes easyopencv
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, cameraX, cameraY, 45, 40);

        // initializes slamra
        Transform2d cameraToRobot = new Transform2d(new Translation2d(6 * 0.0254, 7 * 0.0254), Rotation2d.fromDegrees(-90));
        Pose2d startingPose = new Pose2d(new Translation2d(startingPoseX * 0.0254, startingPoseY * 0.0254), Rotation2d.fromDegrees(90));
        GlobalSlamra.startCamera(hardwareMap, cameraToRobot, startingPose);

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions
        shooterServo.setPosition(1);
        wobbleAxis2.setPosition(0);

        camera.startDetection();
        // loops this until start is pressed
        while (!isStarted()) {
            // gets the current amount of rings
            activeGoal = ringCount(0, camera);
            packet.put("Goal", activeGoal);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Goal", activeGoal);
            telemetry.update();
        }
        camera.stopDetection();

        // passes hardware to slamra class
        DcMotor[] motors = {m1, m2, m3, m4};
        slauto.setUp(motors, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);
    }

    // Function which is called to shoot a given number of rings, at a given speed, with given delays
    public void shoot(double tps, int amount, long rev, long delay, boolean doStop) {
        shooter.setVelocity(tps);
        sleep(rev);
        for (int i = 0; i < amount; i++) {
            shooterServo.setPosition(0.3);
            sleep(300);
            shooterServo.setPosition(1);
            sleep(delay);
        }
        if (doStop) {
            shooter.setVelocity(0);
        }
    }

    // Function which is calleed to synchronously drive to a wobble goal and deploy the wobble
    public void wobbleSync(double speed, String side, int goal, String motion, SimpleSlamra slauto, TeleAuto callback) {
        if (side == "red") {
            if (goal == 0) {
                slauto.drive(20, 69, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-4, 51, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-28, 69, 180, speed, callback);
            }

        } else if (side == "blue") {
            if (goal == 0) {
                slauto.drive(20, -56, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-4, -38, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-28, -56, 180, speed, callback);
            }
        }
        wobbleControl(motion, callback);
    }

    // Function which can be called to drive the wobble grabber to common positions
    public void wobbleControl(String motion, TeleAuto callback) {
        if (motion == "store") {
            wobbleAxis1.setTargetPosition(0);
            wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleAxis1.setPower(1);
            while (callback.opModeIsActive() && wobbleAxis1.isBusy()) sleep(10);

            wobbleAxis1.setPower(0);

        } else if (motion == "drop") {
            wobbleAxis1.setTargetPosition(6500);
            wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleAxis1.setPower(1);
            while (callback.opModeIsActive() && wobbleAxis1.isBusy() && !armTouch.isPressed()) sleep(10);

            wobbleAxis1.setPower(0);
            sleep(100);
            wobbleAxis2.setPosition(0.5);
            sleep(1000);

        } else if (motion == "raise") {
            wobbleAxis1.setTargetPosition(3050);
            wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleAxis1.setPower(1);
            while (callback.opModeIsActive() && wobbleAxis1.isBusy()) sleep(10);

            wobbleAxis1.setPower(0);
            wobbleAxis2.setPosition(0);
        }
    }

    // Function which asynchronously deploys wobble and drives to the goal
    public void wobbleAsync(int position, double power, double speed, String side, int goal, SimpleSlamra slauto, TeleAuto callback) {
        wobbleAxis1.setTargetPosition(position);
        wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleAxis1.setPower(power);

        if (side == "red") {
            if (goal == 0) {
                slauto.drive(18, 70, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-6, 51, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-30, 70, 180, speed, callback);
            }

        } else if (side == "blue") {
            if (goal == 0) {
                slauto.drive(20, -53, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-4, -35, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-28, -53, 180, speed, callback);
            }
        }
    }

    // Function used for the second wobble, if there is one. It places it slightly away from the first
    public void wobbleAsyncSecond(int position, double power, double speed, int goal, SimpleSlamra slauto, TeleAuto callback) {
        wobbleAxis1.setTargetPosition(position);
        wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleAxis1.setPower(power);

        if (goal == 0) {
            slauto.drive(35, 53, 180, speed, 0, callback, false, true);
            slauto.drive(24, 72, 180, speed, callback);
        } else if (goal == 1) {
            slauto.drive(0, 53, 180, speed, callback);
        } else if (goal == 2) {
            slauto.drive(-10, 53, 180, speed, 0, callback, false, true);
            slauto.drive(-24, 72, 180, speed, 0, callback, true, false);
        }
    }

    // Function which simply drives the wobble grabber to a set position, asynchronously
    public void wobbleManual(int position, double power) {
        wobbleAxis1.setTargetPosition(position);
        wobbleAxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleAxis1.setPower(power);
    }

    // Function which simply sets the position of the wobble grabber servo, once the wobble grabber isnt moving.
    public void wobbleMove(boolean down, TeleAuto callback, Telemetry telemetry) {
        ElapsedTime timeout = new ElapsedTime();
        while (callback.opModeIsActive() && wobbleAxis1.isBusy() && timeout.seconds() < 4 && !armTouch.isPressed()) {
            sleep(10);
            telemetry.addData("encoder", wobbleAxis1.getCurrentPosition());
        }
        if (down) {
            wobbleAxis1.setPower(0);
            sleep(100);
            wobbleAxis2.setPosition(0.5);
        } else {
            wobbleAxis1.setPower(0);
            sleep(100);
            wobbleAxis2.setPosition(0);
        }
    }

    // Function which can be used to set both of the intake motors' speeds
    public void intakeControl(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    // Function which can be used to extend the tape measure for a specified time
    public void park(long extendTime) {
        tapeMeasure.setPower(1);
        sleep(extendTime);
        tapeMeasure.setPower(0);
    }

    // Function which handles the amount of rings, and gives it to the op mode
    public int ringCount(long delay, EasyOpenCVImportable camera) {
        int activeGoal = 0;
        sleep(delay);
        EasyOpenCVImportable.RingNumber rings = camera.getDetection();
        if (rings.equals(EasyOpenCVImportable.RingNumber.FOUR)) {
            activeGoal = 2;
            System.out.println("Active Rings: FOUR");
        } else if (rings.equals(EasyOpenCVImportable.RingNumber.ONE)) {
            activeGoal = 1;
            System.out.println("Active Rings: ONE");
        } else if (rings.equals(EasyOpenCVImportable.RingNumber.NONE)) {
            activeGoal = 0;
            System.out.println("Active Rings: NONE");
        }
        return activeGoal;
    }
}

