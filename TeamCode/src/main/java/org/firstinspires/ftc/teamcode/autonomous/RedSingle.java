package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;
import org.firstinspires.ftc.teamcode.zimportants.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;

@Autonomous(name="RedSingle", group="Red")
public class RedSingle extends LinearOpMode implements TeleAuto {

    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;

    private DcMotor wobbleAxis1 = null;
    private Servo wobbleAxis2 = null;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    private DcMotorEx shooter = null;
    private Servo shooterServo = null;
    private CRServo tapeMeasure = null;

    private static T265Camera slamra = null;

    SimpleSlamra slauto = new SimpleSlamra();
    EasyOpenCVImportable camera = new EasyOpenCVImportable();
    AutoImport auto = new AutoImport();
    private BNO055IMU imu;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // vars used in program
    private int activeGoal;

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
        // tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");

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
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 225, 150, 45, 40);

        // initializes slamra
        if (slamra == null) {
            Transform2d cameraToRobot = new Transform2d(new Translation2d(6 * 0.0254, 7 * 0.0254), Rotation2d.fromDegrees(-90));
            Pose2d startingPose = new Pose2d(new Translation2d(31 * 0.0254, -56 * 0.0254), Rotation2d.fromDegrees(90));
            slamra = new T265Camera(cameraToRobot, 0.1, hardwareMap.appContext);
            slamra.setPose(startingPose);
        }

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // passes hardware to auto class
        auto.setUp(shooter, shooterServo, wobbleAxis2, wobbleAxis1, tapeMeasure, intake1, intake2);

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions
        shooterServo.setPosition(1);

        camera.startDetection();
        // loops this until start is pressed
        while (!isStarted()) {
            // gets the current amount of rings
            activeGoal = auto.ringCount(0, camera);
            packet.put("Goal: ", activeGoal);
            dashboard.sendTelemetryPacket(packet);
        }
        camera.stopDetection();

        // passes hardware to slamra class
        DcMotor[] motors = {m1, m2, m3, m4};
        slauto.setUp(motors, slamra, imu, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        if (opModeIsActive()) {
            slamra.start(); // starts slamra

            // spins up flywheel
            shooter.setVelocity(-1350);

            // drives to first power shot and shoots
            slauto.drive(-4, 23, 0, 1, this);
            auto.shoot(-1350, 1, 0, 100, false);

            // drives to second power shot and shoots
            slauto.drive(-4, 18, 0, 1, this);
            auto.shoot(-1350, 1, 0, 100, false);

            // drives to third power shot and shoots
            slauto.drive(-4, 12, 0, 1, this);
            auto.shoot(-1350, 1, 0, 100, true);

            // drives to active goal and places first wobble
            auto.wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
            auto.wobbleMove(true, this);
            sleep(200);
            auto.wobbleManual(3050, 1);
            sleep(500);

            // does the following if there are rings on field
            if (activeGoal == 1) {
                // moves away from wobble
                slauto.drive(-3, 43, 180, 1, this);

                // picks up single ring
                slauto.drive(0, 43, 0, 1, this);
                auto.intakeControl(1);
                slauto.drive(16, 43, 0, 1, this, false, false);
                sleep(2000);

                // drives to shooting position
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);

                auto.intakeControl(0); // turns off intake

                // shoots
                auto.shoot(-1500, 1, 0, 100, true);
                shooter.setVelocity(0);

            } else if (activeGoal == 2) {
                // knocks down stack of rings, and picks 3 up
                slauto.drive(0, 41, 0, 1, this);
                slauto.drive(10, 41, 0, 1, this);
                slauto.drive(0, 41, 0, 1, this);
                auto.intakeControl(1);
                slauto.drive(20, 41, 0, 1, this);
                sleep(1000);
                auto.intakeControl(0);

                // drives to shooting position and shoots
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                auto.shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);

                // picks up remaining ring
                slauto.drive(0, 41, 0, 1, this);
                auto.intakeControl(1);
                slauto.drive(12, 41, 0, 1, this);
                sleep(100);
                auto.intakeControl(0);

                // shoots remaining ring
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                auto.shoot(-1500, 1, 0, 100, true);
                shooter.setVelocity(0);
            }

            // moves second wobble to zone
            slauto.drive(26, 58, 0, 1, this);
            auto.wobbleManual(6600, 1);
            sleep(10000);

            slamra.stop(); // stops slamra
        }
    }
}
