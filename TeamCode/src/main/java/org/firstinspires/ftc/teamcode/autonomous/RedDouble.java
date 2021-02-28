package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedDouble", group="Red")
public class RedDouble extends AutoImport {

    public RedDouble() { super(30, -56, 225, 172); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            if (activeGoal != 2) {
                // drives to shooting position and shoots 3 rings
                shooter.setVelocity(-1600);
                if (activeGoal != 0) {
                    slauto.drive(9, 27, 0, 1, 0, this, false, true);
                }
                slauto.drive(0, 15, -18, 1, this);
                shoot(-1550, 3, 0, 500, true);

                // does the following if there are rings on field
                if (activeGoal != 0) {
                    slauto.drive(10, 15, 0, 1, this);
                    sleep(2000);
                }
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
                    slauto.drive(0, 40, 0, 1, this);
                    slauto.drive(10, 40, 0, 1, 0, this, false, false); // Knocks Stack
                    slauto.drive(7, 40, 0, 1, 0, this, false, false);
                    intakeControl("in");
                    slauto.drive(24, 40, 0, 0.3, 5, this, false, true);
                    sleep(500);
                    intakeControl("off");

                    // drives to shooting position and shoots
                    shooter.setVelocity(-1600);
                    slauto.drive(2, 39, 0, 1, this);
                    shoot(-1500, 3, 0, 500, true);
                    shooter.setVelocity(0);
                }

                slauto.drive(35, 28, 180, 1, this);
                if (activeGoal != 2) {
                    sleep(2000);
                }
                if (activeGoal == 0) {
                    sleep(2500);
                }

                // drives to wobble goal and drops, before raising again
                wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
                wobbleMove(true, this, telemetry);
                sleep(500);
                wobbleManual(3050, 1);
                sleep(200);
                if (activeGoal == 1) {
                    sleep(800);
                }

                // parks
                if (activeGoal != 1) {
                    slauto.drive(0, -7, 180, 1, this);
                } else {
                    slauto.drive(0, 46, 180, 1, this);
                }

                // If there are 4 rings
            } else {
                // drives to shooting position and shoots 3 rings
                shooter.setVelocity(-1600);
                if (activeGoal != 0) { slauto.drive(9, 27, 0, 1, 0, this, false, true); }
                slauto.drive(2, 39, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);

                // knocks down stack of rings, and picks 3 up
                slauto.drive(4, 40, 0, 1, this);
                slauto.drive(10, 40, 0, 1, 0, this, false, false); // Knocks Stack
                slauto.drive(7, 40, 0, 1, 0, this, false, false);
                intakeControl("in");
                slauto.drive(24, 40, 0, 0.3, 5, this, false, true);
                sleep(500);
                intakeControl("off");

                // drives to shooting position and shoots
                shooter.setVelocity(-1600);
                slauto.drive(2, 39, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);

                // drives to wobble goal and drops, before raising again
                wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
                wobbleMove(true, this, telemetry);
                sleep(500);
                wobbleManual(3050, 1);
                sleep(200);

                tapeMeasure.setPower(1); // starts extending tape measure to park
                sleep(4000);
                tapeMeasure.setPower(0);
            }
        }
    }
}