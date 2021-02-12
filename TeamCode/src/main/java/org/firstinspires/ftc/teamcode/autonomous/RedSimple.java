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

import org.firstinspires.ftc.teamcode.vslamcam.SimpleSlamra;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;
import org.firstinspires.ftc.teamcode.zimportants.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;
import org.firstinspires.ftc.teamcode.zimportants.GlobalSlamra;

@Autonomous(name="RedSimple", group="Red")
public class RedSimple extends LinearOpMode implements TeleAuto {

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
        //tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");

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
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 275, 125, 45, 60);

        // initializes slamra
        Transform2d cameraToRobot = new Transform2d(new Translation2d(6 * 0.0254, 7 * 0.0254), Rotation2d.fromDegrees(-90));
        Pose2d startingPose = new Pose2d(new Translation2d(31 * 0.0254, -56 * 0.0254), Rotation2d.fromDegrees(90));
        GlobalSlamra.startCamera(hardwareMap, cameraToRobot, startingPose);

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
        slauto.setUp(motors, imu, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        if (opModeIsActive()) {
            sleep(3000); // Wait 3 seconds before moving

            // Scoots over
            slauto.drive(56, 15, 0, 1, this);

            sleep(14000); // Wait 14 seconds before moving to shoot

            // Drives to shooting area and shoots 3
            shooter.setVelocity(-1500);
            slauto.drive(4, 20, -15, 1, this);
            auto.shoot(-1500, 3, 0, 500, true);

            // Parks
            slauto.drive(-15, 20, -17, 1, this);
        }
    }
}
