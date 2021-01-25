package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Shooter", group="Test")
public class Shooter extends LinearOpMode{

    // defines motors and stuff
    private DcMotor launcher = null;
    private CRServo flicker = null;

    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        flicker = hardwareMap.get(CRServo.class, "flicker");

        waitForStart();

        while (opModeIsActive()) {

            // flicker servo
            if (gamepad1.a) {
                ElapsedTime launchServoTime = new ElapsedTime();
                while(launchServoTime.milliseconds() < 600 && opModeIsActive()) {
                    flicker.setPower(1);
                }
                flicker.setPower(0);
            }

            // launcher motor
            if (gamepad1.right_bumper) {
                launcher.setPower(1);
            } else {
                launcher.setPower(0);
            }
        }
    }
}
