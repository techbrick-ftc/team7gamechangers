package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedSingle", group="Red")
public class RedSingle extends AutoImport {
    
    public RedSingle() { super(31, -56, 225, 150); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            /*// spins up flywheel
            shooter.setVelocity(-1350); // orig -1350

            // drives to first power shot and shoots
            slauto.drive(-4, 23, 0, 1, this);
            shoot(-1350, 1, 0, 100, false);

            // drives to second power shot and shoots
            slauto.drive(-4, 17, 0, 1, this);
            shoot(-1310, 1, 0, 100, false);

            // drives to third power shot and shoots
            slauto.drive(-4, 10, 0, 1, this);
            shoot(-1310, 1, 0, 100, true);*/

            // drives to shooting position and shoots 3 rings
            shooter.setVelocity(-1500);
            slauto.drive(9, 24, 0, 1, 0, this, false, true);
            slauto.drive(2, 39, 0, 1, this);
            shoot(-1500, 3, 0, 500, true);

            // does the following if there are rings on field
            if (activeGoal == 1) {
                // picks up single ring
                slauto.drive(0, 43, 0, 1, this);
                intakeControl(1);
                slauto.drive(16, 43, 0, 1, this);
                sleep(1000);

                // drives to shooting position
                shooter.setVelocity(-1490);
                slauto.drive(2, 39, 0, 1, 0, this, false, false);

                intakeControl(0); // turns off intake

                // shoots
                shoot(-1490, 1, 0, 100, true);
                shooter.setVelocity(0);

            } else if (activeGoal == 2) {
                // knocks down stack of rings, and picks 3 up
                slauto.drive(4, 43, 0, 1, this);
                slauto.drive(10, 43, 0, 1, 0, this, false, false);
                slauto.drive(7, 43, 0, 1, 0, this, false, false);
                intakeControl(1);
                slauto.drive(30, 43, 0, 1, 5, this, true, true);
                sleep(1000);
                intakeControl(0);

                // drives to shooting position and shoots
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);
            }

            // drives to active goal and places first wobble
            wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
            wobbleMove(true, this, telemetry);
            sleep(200);
            wobbleManual(3050, 1);
            sleep(200);

            // grabs second wobble
            slauto.drive(27, 57, 0, 1, this);
            wobbleManual(7300, 1);
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
