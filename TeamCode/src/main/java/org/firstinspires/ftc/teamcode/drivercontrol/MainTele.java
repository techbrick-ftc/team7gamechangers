package org.firstinspires.ftc.teamcode.drivercontrol;

import android.text.method.Touch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;
import org.firstinspires.ftc.teamcode.zimportants.GlobalVars;
import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;
import org.firstinspires.ftc.teamcode.zimportants.GlobalSlamra;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@TeleOp(name="Main", group="Mechanum")
public class MainTele extends LinearOpMode implements TeleAuto{

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    SimpleSlamra slauto = new SimpleSlamra();
    AutoImport auto = new AutoImport();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    private DcMotor wobbleAxis1 = null;
    private Servo wobbleAxis2 = null;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    private DcMotorEx shooter = null;
    private Servo shooterServo = null;
    private CRServo tapeMeasure = null;
    private TouchSensor armTouch = null;

    FieldCentric drive = new FieldCentric();
    private BNO055IMU imu = null;

    private int shooterTPS = -1540;

    public boolean driverAbort() {
        return gamepad1.y;
    }

    public void runOpMode() {
        int loops = 0;
        ElapsedTime launchServoTime = null;

        // adds start telemetry
        telemetry.addLine("hardware ready");
        telemetry.update();

        // configures hardware
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        wobbleAxis1 = hardwareMap.get(DcMotor.class, "wobble_axis_1");
        wobbleAxis2 = hardwareMap.get(Servo.class, "wobble_axis_2");
        intake1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake2 = hardwareMap.get(DcMotor.class, "intake_2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterServo = hardwareMap.get(Servo.class, "shooter_servo");
        tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");
        armTouch = hardwareMap.get(TouchSensor.class, "arm_touch");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleAxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleAxis1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Defines motor configs
        final double PI = Math.PI;
        DcMotor[] motors = {fr, rr, rl, fl};
        double[] motorAngles = {3*PI/4, 5*PI/4, 7*PI/4, PI/4};

        // Sets up motor configs
        try {
            drive.setUp(motors, motorAngles, imu);
        } catch (Exception e) {
            packet.put("SETUP ERROR", true);
            dashboard.sendTelemetryPacket(packet);
            e.printStackTrace();
        }

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Configures Variables
        boolean axis2Switch = false;
        boolean intakeSwitch = false;

        // Anything that moves in init
        shooterServo.setPosition(1);
        wobbleAxis2.setPosition(0);

        // initializes slamra
        Transform2d cameraToRobot = new Transform2d(new Translation2d(6 * 0.0254, 7 * 0.0254), Rotation2d.fromDegrees(-90));
        Pose2d startingPose = new Pose2d(new Translation2d(31 * 0.0254, -56 * 0.0254), Rotation2d.fromDegrees(90));
        GlobalSlamra.startCamera(hardwareMap, cameraToRobot, startingPose);

        waitForStart();

        slauto.setUp(motors, imu, telemetry);
        auto.setUp(shooter, shooterServo, wobbleAxis2, wobbleAxis1, tapeMeasure, intake1, intake2, armTouch);

        // Robot Control
        while(opModeIsActive()) {
            // Updates cur1 & 2
            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            packet.put("cur2", cur2.x);
            packet.put("trigger", gamepad2.right_trigger);
            dashboard.sendTelemetryPacket(packet);

            // Set wheel powers
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // Wobble Grabber Control
            double wobbleY = gamepad2.left_stick_y;
            double wobble1Power = Range.clip(wobbleY, -0.75, 0.75);
            if (armTouch.isPressed()) {
                wobble1Power = Range.clip(wobbleY, -0.75, 0);
            }
            wobbleAxis1.setPower(wobble1Power);

            if (cur2.b && !prev2.b && !axis2Switch) {
                wobbleAxis2.setPosition(0.5);
                axis2Switch = true;
            } else if (cur2.b && !prev2.b && axis2Switch) {
                wobbleAxis2.setPosition(0);
                axis2Switch = false;
            }

            // Intake Control
            if (cur2.right_bumper && !prev2.right_bumper && !intakeSwitch) {
                intake1.setPower(1);
                intake2.setPower(1);
                intakeSwitch = true;
            } else if (cur2.right_bumper && !prev2.right_bumper && intakeSwitch) {
                intake1.setPower(0);
                intake2.setPower(0);
                intakeSwitch = false;
            } else if (gamepad2.left_bumper) {
                intake1.setPower(-1);
                intake2.setPower(-1);
                intakeSwitch = true;
            }

            // Shooter Control
            if (gamepad2.right_trigger > 0.1) {
                shooter.setVelocity(shooterTPS);
            } else {
                shooter.setVelocity(0);
            }

            // Shooter Servo Control
            if (launchServoTime == null && cur2.a && !prev2.a) {
                launchServoTime = new ElapsedTime();
                shooterServo.setPosition(0.3);
            } else if (launchServoTime != null && launchServoTime.milliseconds() > 600) {
                launchServoTime = null;
            } else if (launchServoTime != null && launchServoTime.milliseconds() > 300) {
                shooterServo.setPosition(1);
            }

            // Tape Measure Control
            if (cur2.x) {
                tapeMeasure.setPower(1);
            } else if (cur2.y) {
                tapeMeasure.setPower(-1);
            } else {
                tapeMeasure.setPower(0);
            }

            // Drive to High Shots
            if (cur1.x) {
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                auto.shoot(-1500, 3, 0, 500, true);
            }

            // Drive to Power Shots
            if (cur1.b) {
                // spins up flywheel
                shooter.setVelocity(-1350); // orig -1350

                // drives to first power shot and shoots
                slauto.drive(-4, 23, 0, 1, this);
                auto.shoot(-1350, 1, 0, 100, false);

                // drives to second power shot and shoots
                slauto.drive(-4, 17, 0, 1, this);
                auto.shoot(-1310, 1, 0, 100, false);

                // drives to third power shot and shoots
                slauto.drive(-4, 10, 0, 1, this);
                auto.shoot(-1310, 1, 0, 100, true);
            }

            // Reset Field Centric button
            if (cur1.a && !prev1.a) {
                drive.newOffset();
            }

            // High shot button
            if (cur2.dpad_up && !prev2.dpad_up) {
                shooterTPS = -1540;
            }

            // Power shot Button
            if (cur2.dpad_down && !prev2.dpad_down) {
                shooterTPS = -1390;
            }

            // Updates prev1 & 2
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Send FTC Dashboard Packets
            packet.put("loop count", loops);
            packet.put("fr power", fr.getPower());
            packet.put("fl power", fl.getPower());
            packet.put("rr power", rr.getPower());
            packet.put("rl power", rl.getPower());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("wobble encoder", wobbleAxis1.getCurrentPosition());
            telemetry.addData("wobble servo position", wobbleAxis2.getPosition());
            telemetry.update();

            loops++;
        }

        /*slamra.stop();*/
    }
}
