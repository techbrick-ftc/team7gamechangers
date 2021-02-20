package org.firstinspires.ftc.teamcode.vslamcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="SimpleSlamraDrive", group="Test")
public class SimpleSlamraDrive extends AutoImport {

    public SimpleSlamraDrive() { super(31, -56, 225, 150); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            slauto.drive(2, 39, 0, 1, this);
        }
    }
}
