package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedSingle", group="Red")
public class RedSingle extends AutoImport {
    
    public RedSingle() { super(30, -56, 225, 150); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // drives to shooting position and shoots 3 rings
            if (activeGoal != 0) { slauto.drive(9, 27, 0, 1, 0, this, false, true); }
            slauto.drive(2, 39, 0, 1, this);
            shoot(-1500, 3, 1000, 500, true);

            // does the following if there are rings on field
            if (activeGoal == 1) {
                // picks up single ring
                slauto.drive(0, 43, 0, 1, this);
                intakeControl(-1);
                slauto.drive(16, 43, 0, 1, this);
                sleep(1000);

                intakeControl(0); // turns off intake

                // drives to shooting position
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, 0, this, false, false);

                // shoots
                shoot(-1490, 1, 10, 100, true);
                shooter.setVelocity(0);

            } else if (activeGoal == 2) {
                // knocks down stack of rings, and picks 3 up
                slauto.drive(4, 43, 0, 1, this);
                slauto.drive(10, 43, 0, 1, 0, this, false, false);
                slauto.drive(7, 43, 0, 1, 0, this, false, false);
                intakeControl(1);
                slauto.drive(30, 43, 0, 1, 5, this, true, true);
                sleep(1000);
                intakeControl(-0);

                // drives to shooting position and shoots
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);
            }

            // drives to wobble goal and drops, before raising again
            wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(1000);
            wobbleManual(3050, 1);
            sleep(200);

            // grabs second wobble
            slauto.drive(27, 57, 0, 1, this);
            wobbleManual(7500, 1);
            wobbleMove(false, this, telemetry);
            sleep(200);
            wobbleManual(3050, 1);

            if (activeGoal == 2) {
                tapeMeasure.setPower(1); // starts extending tape measure to park
            }

            // moves second wobble to zone
            wobbleAsyncSecond(6500, 1, 1, activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(200);
            wobbleManual(3050, 1);
            sleep(200);

            // parks
            if (activeGoal == 0) {
                slauto.drive(35, 53, 180, 1, 0, this, false, true);
                slauto.drive(6, 51, 180, 1, this);
            } else {
                slauto.drive(6, 51, 180, 1, this);
            }
        }
    }
}
