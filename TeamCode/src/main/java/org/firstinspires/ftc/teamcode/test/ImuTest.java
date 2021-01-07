package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="imutest", group="test")
public class ImuTest extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    private BNO055IMU imu;
    private BNO055IMU imu1;
    //private BNO055IMU imu2;

    public void runOpMode() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
        //imu2 = hardwareMap.get(BNO055IMU.class, "imu 2");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        imu.initialize(params);
        imu1.initialize(params);
        //imu2.initialize(params);

        waitForStart();

        while (opModeIsActive()) {

            packet.put("IMU Z", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            packet.put("IMU Z I", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            packet.put("IMU1 Z", imu1.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            packet.put("IMU1 Z I", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //packet.put("IMU2 Z", imu2.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //packet.put("IMU2 Z I", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            dash.sendTelemetryPacket(packet);
        }
    }
}
