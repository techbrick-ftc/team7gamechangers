package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedRight", group="Red")
public class RedRight extends AutoImport {

    public RedRight() { super(53, -56, 25, 150); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // drives to shooting position and shoots 3 rings
            shooter.setVelocity(-1500);
            slauto.drive(9, 24, 0, 1, 0, this, false, true);
            slauto.drive(2, 39, 0, 1, this);
            shoot(-1500, 3, 0, 500, true);

            // drives to wobble goal and drops, before raising again
            wobbleSync(1, "red", activeGoal, "drop", slauto, this);
            wobbleControl("store", this);

            // parks at middle of field
            slauto.drive(-10, 15, -90, 1, this);
        }
    }
}
