package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zimportants.EasyOpenCVImportable;

@TeleOp(name="webcamTest", group="Test")
public class WebcamTest extends LinearOpMode {
    EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        easyOpenCVImportable.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 30, 125, 45, 60);

        waitForStart();

        while(opModeIsActive()) {
            easyOpenCVImportable.startDetection();
            while(opModeIsActive() && !isStopRequested()) {
                packet.put("status", easyOpenCVImportable.getDetecting());
                packet.put("detected", easyOpenCVImportable.getDetection());
                packet.put("analysis", easyOpenCVImportable.getAnalysis());
                dashboard.sendTelemetryPacket(packet);
                dashboard.startCameraStream(easyOpenCVImportable.getWebCamera(), 0);
            }
        }
    }
}