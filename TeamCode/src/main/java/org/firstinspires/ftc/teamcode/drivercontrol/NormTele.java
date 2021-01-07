package org.firstinspires.ftc.teamcode.drivercontrol;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Main", group="Mechanum")
public class NormTele extends LinearOpMode{

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    private DcMotor wobbleAxis1 = null;
    private Servo wobbleAxis2 = null;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    private DcMotor shooter = null;
    private Servo shooterServo = null;

    public void runOpMode() {
        int loops = 0;

        // adds start telemetry
        telemetry.addData("status", "initialized");
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
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterServo = hardwareMap.get(Servo.class, "shooter_servo");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setDirection(DcMotor.Direction.FORWARD);
        rl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*// Defines motor configs
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
        }*/

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Configures Switches
        boolean axis2Switch = false;
        boolean intakeSwitch = false;

        // Anything that moves in init
        shooterServo.setPosition(1);
        wobbleAxis2.setPosition(0);

        waitForStart();

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

            // Set wheel powers
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double flPower = Range.clip(drive + strafe + rotate, -1.0, 1.0);
            double frPower = Range.clip(drive - strafe - rotate, -1.0, 1.0);
            double rlPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);
            double rrPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);

            fl.setPower(flPower);
            fr.setPower(frPower);
            rl.setPower(rlPower);
            rr.setPower(rrPower);

            // Wobble Grabber Control
            double wobbleY = gamepad2.left_stick_y;
            double wobble1Power = Range.clip(wobbleY, -0.75, 0.75);

            wobbleAxis1.setPower(wobble1Power);
            wobbleAxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleAxis1.setDirection(DcMotorSimple.Direction.REVERSE);

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
            /*if (cur2.a && !prev2.a) {
                ElapsedTime launchTime = new ElapsedTime();
                while(launchTime.milliseconds() < 600 && opModeIsActive() && !isStopRequested()) {
                    shooterServo.setPower(1);
                    shooter.setPower(1);
                }
                shooterServo.setPower(0);
                shooter.setPower(0);
            }*/

            // Shooter Servo Control
            if (cur2.a && !prev2.a) {
                ElapsedTime launchServoTime = new ElapsedTime();
                while(launchServoTime.milliseconds() < 1000 && opModeIsActive()) {
                    shooterServo.setPosition(-1);
                }
                shooterServo.setPosition(1);
            }

            // Shooter Control
            //double shooterR = shooter.getCurrentPosition() / 1000;
            //double shooterRPS = shooterR / runtime.seconds();


            if (gamepad2.right_trigger > 0.1) {
                shooter.setPower(1);
            } else {
                shooter.setPower(0);
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

            loops++;
        }
    }
}
