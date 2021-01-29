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
import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;

@Autonomous(name="RedLeft", group="Red")
public class RedLeft extends LinearOpMode implements TeleAuto {

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
        tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");

        wobbleAxis1.setDirection(DcMotorSimple.Direction.REVERSE);

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
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap);

        // initializes slamra
        if (slamra == null) {
            Transform2d cameraToRobot = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90));
            Pose2d startingPose = new Pose2d(new Translation2d(23 * 0.0254, -62 * 0.0254), Rotation2d.fromDegrees(90));
            slamra = new T265Camera(cameraToRobot, 0.1, hardwareMap.appContext);
            slamra.setPose(startingPose);
        }

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // passes hardware to auto class
        auto.setUp(shooter, shooterServo, wobbleAxis2, wobbleAxis1, tapeMeasure);

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions
        shooterServo.setPosition(1);

        waitForStart();

        // passes hardware to slamra class
        DcMotor[] motors = {m1, m2, m3, m4};
        slauto.setUp(motors, slamra, imu, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        if (opModeIsActive()) {
            slamra.start(); // starts slamra
            auto.wobbleControl("raise", this); // moves wobble out of slamra's way

            // gets the current amount of rings
            camera.startDetection();
            sleep(1000);
            EasyOpenCVImportable.RingNumber rings = camera.getDetection();
            camera.stopDetection();
            if (rings.equals(EasyOpenCVImportable.RingNumber.FOUR)) {
                activeGoal = 2;
            } else if (rings.equals(EasyOpenCVImportable.RingNumber.ONE)) {
                activeGoal = 1;
            } else if (rings.equals(EasyOpenCVImportable.RingNumber.NONE)) {
                activeGoal = 0;
            }

            // drives to shooting position and shoots 3
            shooter.setVelocity(-1540);
            slauto.drive(7, 39, 0, 1, this);
            auto.shoot(-1540, 3, 0, 1500);

            // drives to wobble goal and drops, before raising again
            auto.wobble("red", activeGoal, "drop", slauto, this);
            auto.wobbleControl("store", this);

            // parks at line
            slauto.drive(7, 52, 0, 1, this);
            auto.park(5000);

            slamra.stop(); // stops slamra
        }
    }
}
