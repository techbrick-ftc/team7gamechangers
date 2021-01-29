package org.firstinspires.ftc.teamcode.vslamcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;

@Disabled
@Autonomous(name="SCAuto", group="Autonomous")
public class SimpleSlamraDrive extends LinearOpMode implements TeleAuto {

    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;

    private static T265Camera slamra = null;

    SimpleSlamra slauto = new SimpleSlamra();
    private BNO055IMU imu;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(param);

        telemetry.addLine("IMU Done");
        telemetry.update();

        if (slamra == null) {
            // This one specifies the location of the camera relative to the robot. We need to specify that it
            // is at 90 degrees, since we still want the front to be on the other side.
            Transform2d cameraToRobot = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90));

            // This one specifies the starting location of the robot on the field, so we will have different values for
            // different starting points on the field.
            //Pose2d startingPose = new Pose2d(new Translation2d(40 * 0.0254, 27 * 0.0254), Rotation2d.fromDegrees(-90));
            Pose2d startingPose = new Pose2d(new Translation2d(23 * 0.0254, -62 * 0.0254), Rotation2d.fromDegrees(90));

            slamra = new T265Camera(cameraToRobot, 0.1, hardwareMap.appContext);
            slamra.setPose(startingPose);
        }

        telemetry.addLine("Camera Done");
        telemetry.update();

        DcMotor[] motors = {m1, m2, m3, m4};

        // adds start telemetry
        telemetry.addData("status", "initialized");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        slauto.setUp(motors, slamra, imu, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        if (opModeIsActive()) {
            slamra.start();
            slauto.drive(-40, 40, 90, 0.8, this);
            slamra.stop();
        }
    }
}
