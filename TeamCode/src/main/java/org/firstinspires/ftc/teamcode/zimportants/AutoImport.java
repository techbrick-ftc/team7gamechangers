package org.firstinspires.ftc.teamcode.zimportants;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;
import org.firstinspires.ftc.teamcode.vslamcentric.SlamraAuto;
import org.firstinspires.ftc.teamcode.vslamcentric.SlamraDrive;

public class AutoImport {

    // Defines vars
    private DcMotorEx shooter;
    private Servo loader;
    private Servo wobbleServo;
    private DcMotor wobbleMotor;
    private CRServo tapeMeasure;
    private DcMotor intake1;
    private DcMotor intake2;
    private TouchSensor armTouch;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotorEx shooter, Servo loader, Servo wobbleServo, DcMotor wobbleMotor, CRServo tapeMeasure, DcMotor intake1, DcMotor intake2, TouchSensor armTouch) {
        this.shooter = shooter;
        this.loader = loader;
        this.wobbleServo = wobbleServo;
        this.wobbleMotor = wobbleMotor;
        this.tapeMeasure = tapeMeasure;
        this.intake1 = intake1;
        this.intake2 = intake2;
        this.armTouch = armTouch;
    }

    // Function which is called to shoot a given number of rings, at a given speed, with given delays
    public void shoot(double tps, int amount, long rev, long delay, boolean doStop) {
        shooter.setVelocity(tps);
        sleep(rev);
        for (int i = 0; i < amount; i++) {
            loader.setPosition(0.3);
            sleep(300);
            loader.setPosition(1);
            sleep(delay);
        }
        if (doStop) {
            shooter.setVelocity(0);
        }
    }

    // Function which is calleed to synchronously drive to a wobble goal and deploy the wobble
    public void wobbleSync(double speed, String side, int goal, String motion, SimpleSlamra slauto, TeleAuto callback) {
        wobbleControl(motion, callback);
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
                slauto.drive(20, -69, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-4, -51, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-28, -69, 180, speed, callback);
            }
        }
    }

    // Function which can be called to drive the wobble grabber to common positions
    public void wobbleControl(String motion, TeleAuto callback) {
        if (motion == "store") {
            wobbleMotor.setTargetPosition(0);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

            wobbleMotor.setPower(0);

        } else if (motion == "drop") {
            wobbleMotor.setTargetPosition(6500);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy() && !armTouch.isPressed()) sleep(10);

            wobbleMotor.setPower(0);
            sleep(100);
            wobbleServo.setPosition(0.5);
            sleep(1000);

        } else if (motion == "raise") {
            wobbleMotor.setTargetPosition(3050);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

            wobbleMotor.setPower(0);
            wobbleServo.setPosition(0);
        }
    }

    // Function which asynchronously deploys wobble and drives to the goal
    public void wobbleAsync(int position, double power, double speed, String side, int goal, SimpleSlamra slauto, TeleAuto callback) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(power);

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
                slauto.drive(18, -68, 180, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-6, -49, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-30, -68, 180, speed, callback);
            }
        }
    }

    // Function used for the second wobble, if there is one. It places it slightly away from the first
    public void wobbleAsyncSecond(int position, double power, double speed, int goal, SimpleSlamra slauto, TeleAuto callback) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(power);

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
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(power);
    }

    // Function which simply sets the position of the wobble grabber servo, once the wobble grabber isnt moving.
    public void wobbleMove(boolean down, TeleAuto callback, Telemetry telemetry) {
        ElapsedTime timeout = new ElapsedTime();
        while (callback.opModeIsActive() && wobbleMotor.isBusy() && timeout.seconds() < 4 && !armTouch.isPressed()) {
            sleep(10);
            telemetry.addData("encoder", wobbleMotor.getCurrentPosition());
        }
        if (down) {
            wobbleMotor.setPower(0);
            sleep(100);
            wobbleServo.setPosition(0.5);
        } else {
            wobbleMotor.setPower(0);
            sleep(100);
            wobbleServo.setPosition(0);
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

    // A class-side sleep function
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

