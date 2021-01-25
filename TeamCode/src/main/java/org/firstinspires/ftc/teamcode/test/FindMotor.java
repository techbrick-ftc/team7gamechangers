package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous(name="Find Motor", group="Test")
public class FindMotor extends LinearOpMode{

    // defines motors and stuff
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fr = null;
    private DcMotor fl = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        waitForStart(); //

        if(opModeIsActive()) {
            fr.setPower(1);
            sleep(1000);
            fr.setPower(0);
            sleep(1000);
            fl.setPower(1);
            sleep(1000);
            fl.setPower(0);
            sleep(1000);
            rl.setPower(1);
            sleep(1000);
            rl.setPower(0);
            sleep(1000);
            rr.setPower(1);
            sleep(1000);
            rr.setPower(0);
        }
    }
}
