package org.firstinspires.ftc.teamcode.zimportants;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotorEx shooter, Servo loader, Servo wobbleServo, DcMotor wobbleMotor, CRServo tapeMeasure, DcMotor intake1, DcMotor intake2) {
        this.shooter = shooter;
        this.loader = loader;
        this.wobbleServo = wobbleServo;
        this.wobbleMotor = wobbleMotor;
        this.tapeMeasure = tapeMeasure;
        this.intake1 = intake1;
        this.intake2 = intake2;
    }

    public void shoot(double tps, int amount, long rev, long delay, boolean doStop) {
        shooter.setVelocity(tps);
        sleep(rev);
        for (int i = 0; i < amount; i++) {
            loader.setPosition(0);
            sleep(500);
            loader.setPosition(1);
            sleep(delay);
        }
        if (doStop) {
            shooter.setVelocity(0);
        }
    }

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
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);

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

    public void wobbleAsync(int position, double power, double speed, String side, int goal, SimpleSlamra slauto, TeleAuto callback) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(power);

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

    public void wobbleManual(int position, double power) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setPower(power);
    }

    public void wobbleMove(boolean down, TeleAuto callback) {
        if (down) {
            while (callback.opModeIsActive() && wobbleMotor.isBusy()) sleep(10);
            wobbleMotor.setPower(0);
            sleep(100);
            wobbleServo.setPosition(0.5);
        } else {
            wobbleServo.setPosition(0);
        }
    }

    public void intakeControl(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void park(long extendTime) {
        tapeMeasure.setPower(1);
        sleep(extendTime);
        tapeMeasure.setPower(0);
    }

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

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

