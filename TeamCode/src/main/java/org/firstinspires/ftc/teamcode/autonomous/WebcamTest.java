package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.EasyOpenCVImportable;

@TeleOp(name="webcamTest", group="Test")
public class WebcamTest extends LinearOpMode {
    EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        easyOpenCVImportable.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            easyOpenCVImportable.startDetection();
            while(opModeIsActive() && !isStopRequested()) {
                packet.put("status", easyOpenCVImportable.getDetecting());
                packet.put("detected", easyOpenCVImportable.getDetection());
                dashboard.sendTelemetryPacket(packet);
                dashboard.startCameraStream(easyOpenCVImportable.getWebCamera(), 0);
            }
        }
    }
}
