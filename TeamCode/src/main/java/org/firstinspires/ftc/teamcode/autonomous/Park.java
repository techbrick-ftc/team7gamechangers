package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="Park", group="Multi")
public class Park extends LinearOpMode {

    private CRServo tapeMeasure = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {

        tapeMeasure = hardwareMap.get(CRServo.class, "tape_measure");

        telemetry.addData("status", "initialized");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        if (opModeIsActive()) {
            sleep(19000);
            tapeMeasure.setPower(1);
            sleep(10000);
            tapeMeasure.setPower(0);
        }
    }
}