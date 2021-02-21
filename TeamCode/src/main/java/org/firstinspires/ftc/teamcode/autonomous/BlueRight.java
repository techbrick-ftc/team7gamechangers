package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="BlueRight", group="Blue")
public class BlueRight extends AutoImport {

    public BlueRight() {
        super(-19, -56, 35, 150);
    }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // drives to shooting position and shoots 3 rings
            if (activeGoal != 0) { slauto.drive(9, -15, 0, 1, 0, this, false, true); }
            slauto.drive(2, -32, 0, 1, this);
            shoot(-1500, 3, 1000, 500, true);

            // drives to wobble goal and drops, before raising again
            wobbleAsync(6500, 1, 1, "blue", activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(1000);
            wobbleManual(3050, 1);
            sleep(200);

            // parks at middle of field
            slauto.drive(-6, -1, -90, 1, this);
        }
    }
}