package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="BlueDouble", group="Blue")
public class BlueDouble extends AutoImport {

    public BlueDouble() {
        super(-19, -56, 35, 172);
    }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            if (activeGoal != 2) {
                // drives to shooting position and shoots 3 rings
                shooter.setVelocity(-1600);
                if (activeGoal != 0) {
                    slauto.drive(9, -13, 0, 1, 0, this, false, true);
                }
                slauto.drive(-1, -4, 20, 1, this);
                shoot(-1550, 3, 0, 500, true);

                // does the following if there are rings on field
                if (activeGoal != 0) {
                    slauto.drive(10, -4, 0, 1, this);
                    sleep(2000);
                }
                if (activeGoal == 1) {
                    // picks up single ring
                    slauto.drive(0, -30, 0, 1, this);
                    intakeControl("in");
                    slauto.drive(17, -30, 0, 0.3, 5, this, false, true);
                    sleep(500);
                    intakeControl("off");

                    // drives to shooting position
                    shooter.setVelocity(-1500);
                    slauto.drive(2, -28, 0, 1, 0, this, false, false);

                    // shoots
                    shoot(-1500, 1, 1000, 100, true);
                    shooter.setVelocity(0);

                } else if (activeGoal == 2) {
                    // knocks down stack of rings, and picks 3 up
                    slauto.drive(0, -29, 0, 1, this);
                    slauto.drive(10, -29, 0, 1, 0, this, false, false); // Knocks Stack
                    slauto.drive(7, -29, 0, 1, 0, this, false, false);
                    intakeControl("in");
                    slauto.drive(24, -29, 0, 0.3, 5, this, false, true);
                    sleep(500);
                    intakeControl("off");

                    // drives to shooting position and shoots
                    shooter.setVelocity(-1600);
                    slauto.drive(2, -28, 0, 1, this);
                    shoot(-1500, 3, 0, 500, true);
                    shooter.setVelocity(0);
                }

                slauto.drive(35, -20, 180, 1, this);
                if (activeGoal != 2) {
                    sleep(2000);
                }
                if (activeGoal == 0) {
                    sleep(2500);
                }

                // drives to wobble goal and drops, before raising again
                wobbleAsync(6500, 1, 1, "blue", activeGoal, slauto, this);
                wobbleMove(true, this, telemetry);
                sleep(500);
                wobbleManual(3050, 1);
                sleep(200);
                if (activeGoal == 1) {
                    sleep(800);
                }

                // parks
                if (activeGoal != 1) {
                    slauto.drive(0, 0, 180, 1, this);
                } else {
                    slauto.drive(-4, -24, 180, 1, this);
                }

                // If there are 4 rings
            } else {
                // drives to shooting position and shoots 3 rings
                shooter.setVelocity(-1600);
                if (activeGoal != 0) { slauto.drive(9, -13, 0, 1, 0, this, false, true); }
                slauto.drive(2, -28, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);

                // knocks down stack of rings, and picks 3 up
                slauto.drive(4, -29, 0, 1, this);
                slauto.drive(10, -29, 0, 1, 0, this, false, false); // Knocks Stack
                slauto.drive(7, -29, 0, 1, 0, this, false, false);
                intakeControl("in");
                slauto.drive(24, -29, 0, 0.3, 5, this, false, true);
                sleep(500);
                intakeControl("off");

                // drives to shooting position and shoots
                shooter.setVelocity(-1600);
                slauto.drive(2, -28, 0, 1, this);
                shoot(-1500, 3, 0, 500, true);
                shooter.setVelocity(0);

                // drives to wobble goal and drops, before raising again
                wobbleAsync(6500, 1, 1, "blue", activeGoal, slauto, this);
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