package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@Autonomous(name="RedDouble", group="Red")
public class RedDouble extends AutoImport {

    public RedDouble() { super(30, -56, 225, 150); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            if (activeGoal == 0) {
                wobbleAsync(6500, 1, 1, "red", activeGoal, slauto, this);
                wobbleMove(true, this, telemetry);
                sleep(1000);
                wobbleManual(3050, 1);
                sleep(200);

                slauto.drive(28, 59, 180, 1, 0, this, false, true);
                shooter.setVelocity(-1550);
                slauto.drive(0, 15, -18, 1, 0, this, true, false);
                shoot(-1550, 3, 0, 500, true);


                // parks at middle of field
                slauto.drive(-6, 5, -90, 1, this);
            } else if (activeGoal == 1) {
                slauto.drive(5, 5, 90, 0, 1, this, false, true);
                wobbleManual(6500, 1);
                slauto.drive(-30, 6, 90, 1, this);
                wobbleMove(true, this, telemetry);
                sleep(1000);
                wobbleManual(3050, 1);
                sleep(200);

                slauto.drive(28, 59, 180, 1, 0, this, false, true);
                shooter.setVelocity(-1550);
                slauto.drive(0, 15, -18, 1, 0, this, true, false);
                shoot(-1550, 3, 0, 500, true);


                /*// parks at middle of field
                slauto.drive(-6, 5, -90, 1, this);*/
            }
        }
    }
}