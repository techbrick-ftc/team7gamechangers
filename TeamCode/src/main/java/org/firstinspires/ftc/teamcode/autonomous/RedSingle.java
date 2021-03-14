package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedSingle", group="Red")
public class RedSingle extends AutoImport {
    
    public RedSingle() { super(30, -56, 225, 172); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            resetEnc(wobbleAxis1);

            // drives to shooting position and shoots 3 rings
            shooter.setVelocity(-1600);
            if (activeGoal != 0) { slauto.drive(9, 27, 0, 1, 0, this, false, true); }
            slauto.drive(2, 39, 0, 1, this);
            shoot(-1500, 3, 0, 500, true);

            // does the following if there are rings on field
            if (activeGoal == 1) {
                // picks up single ring
                slauto.drive(0, 40, 0, 1, this);
                intakeControl("in");
                slauto.drive(17, 40, 0, 0.3, 5, this, false, true);
                sleep(500);
                intakeControl("off");

                // drives to shooting position
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, 0, this, false, false);

                // shoots
                shoot(-1500, 1, 1000, 100, true);
                shooter.setVelocity(0);

            } else if (activeGoal == 2) {
                // knocks down stack of rings, and picks 3 up
                slauto.drive(4, 40, 0, 1, this);
                slauto.drive(10, 40, 0, 1, 0, this, false, false); // Knocks Stack
                slauto.drive(6, 40, 0, 1, 0, this, false, false);
                intakeControl("in");
                slauto.drive(24, 40, 0, 0.3, 5, this, false, true);

                // drives to shooting position and shoots
                shooter.setVelocity(-1600);
                slauto.drive(2, 39, 0, 1, this);
                intakeControl("off");
                shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);
            }

            // drives to wobble goal and drops, before raising again
            wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(500);
            wobbleManual(3050, 1);
            sleep(200);
            if (activeGoal == 0) { slauto.drive(30, 66, 180, 1, 0, this, false, true); }

            // grabs second wobble
            slauto.drive(28, 56, 0, 1, this);
            wobbleManual(7600, 1);
            wobbleMove(false, this, telemetry);
            sleep(200);
            wobbleManual(3050, 1);

            if (activeGoal == 2) {
                tapeMeasure.setPower(1); // starts extending tape measure to park
            }

            // moves second wobble to zone
            wobbleAsyncSecond(6500, 1, 1, "red", activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(500);
            wobbleManual(3050, 1);
            sleep(200);
            if (activeGoal == 2) { sleep(2800); }

            // parks
            if (activeGoal == 0) {
                slauto.drive(35, 53, 180, 1, 0, this, false, true);
                slauto.drive(6, 45, 180, 1, this);
            } else if (activeGoal == 1) {
                slauto.drive(6, 45, 180, 1, this);
            }
        }
    }
}
