package org.firstinspires.ftc.teamcode.zimportants;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;

public class AutoImport {

    // Defines vars
    private DcMotorEx shooter;
    private Servo loader;
    private Servo wobbleServo;
    private DcMotor wobbleMotor;
    private CRServo tapeMeasure;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotorEx shooter, Servo loader, Servo wobbleServo, DcMotor wobbleMotor, CRServo tapeMeasure) {
        this.shooter = shooter;
        this.loader = loader;
        this.wobbleServo = wobbleServo;
        this.wobbleMotor = wobbleMotor;
        this.tapeMeasure = tapeMeasure;
    }

    public void shoot(double tps, int amount, long rev, long delay) {
        shooter.setVelocity(tps);
        sleep(rev);
        for (int i = 0; i < amount; i++) {
            loader.setPosition(0);
            sleep(500);
            loader.setPosition(1);
            sleep(delay);
        }
        shooter.setVelocity(0);
    }

    public void wobble(double speed, String side, int goal, String motion, SimpleSlamra slauto, TeleAuto callback) {
        if (side == "red") {
            if (goal == 0) {
                slauto.drive(6, 48, 180, speed, callback);;
            } else if (goal == 1) {
                slauto.drive(-18, 28, 180, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-42, 52, 180, speed, callback);
            }

        } else if (side == "blue") {
            if (goal == 0) {
                slauto.drive(6, -48, 0, speed, callback);
            } else if (goal == 1) {
                slauto.drive(-18, -28, 0, speed, callback);
            } else if (goal == 2) {
                slauto.drive(-42, -52, 0, speed, callback);
            }
        }

        wobbleControl(motion, callback);
    }

    public void wobbleControl(String motion, TeleAuto callback) {
        if (motion == "store") {
            wobbleServo.setPosition(0);
            wobbleMotor.setTargetPosition(0);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

            wobbleMotor.setPower(0);

        } else if (motion == "drop") {
            wobbleMotor.setTargetPosition(6100);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

            wobbleMotor.setPower(0);
            wobbleServo.setPosition(0.5);
            sleep(1000);

        } else if (motion == "raise") {
            wobbleServo.setPosition(0);
            wobbleMotor.setTargetPosition(3050);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(1);
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

            wobbleMotor.setPower(0);
        }
    }

    public void park(long extendTime) {
        tapeMeasure.setPower(1);
        sleep(extendTime);
        tapeMeasure.setPower(0);
    }

    public int ringCount(long delay, EasyOpenCVImportable camera) {
        int activeGoal = 0;
        camera.startDetection();
        sleep(delay);
        EasyOpenCVImportable.RingNumber rings = camera.getDetection();
        camera.stopDetection();
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

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

