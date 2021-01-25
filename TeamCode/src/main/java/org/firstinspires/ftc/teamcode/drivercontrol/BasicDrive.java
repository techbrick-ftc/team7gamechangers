package org.firstinspires.ftc.teamcode.drivercontrol;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Basic Drive", group="Mechanum")
public class BasicDrive extends LinearOpMode{

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // defines motors and stuff
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    private DcMotor wobbleAxis1 = null;
    private Servo wobbleAxis2 = null;
    /*private DcMotor shooter = null;
    private DcMotor intake = null;*/

    @Override
    public void runOpMode() {

        int loops = 0;

        // adds start telemetry
        telemetry.addData("status", "initialized");
        telemetry.update();

        // configures hardware
        /*fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        rl = hardwareMap.get(DcMotor.class, "back_left_motor");
        rr = hardwareMap.get(DcMotor.class, "back_right_motor");*/

        wobbleAxis1 = hardwareMap.get(DcMotor.class, "wobble_axis_1");
        wobbleAxis2 = hardwareMap.get(Servo.class, "wobble_axis_2");
        /*shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");*/

        // configures motor directions
        /*fl.setDirection(DcMotor.Direction.FORWARD);
        rl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);*/

        // configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // configures switch variables
        /*boolean shooterSwitch = false;*/
        boolean axis2Switch = false;

        // waits for start
        waitForStart();
        runtime.reset();

        // after start is pressed
        while(opModeIsActive()) {

            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                idle();
            }

            // wobble grabber
            double wobble1Power;
            double wobbleY = gamepad1.left_stick_y;

            wobble1Power = Range.clip(wobbleY, -0.75, 0.75);

            wobbleAxis1.setPower(wobble1Power);
            wobbleAxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobbleAxis1.setDirection(DcMotorSimple.Direction.REVERSE);

            if (cur1.b && !prev1.b) {
                packet.put("Pressed B", true);
            } else {
                packet.put("Pressed B", false);
            }

            if (cur1.b && !prev1.b && !axis2Switch) {
                wobbleAxis2.setPosition(0.5);
                axis2Switch = true;
            } else if (cur1.b && !prev1.b && axis2Switch) {
                wobbleAxis2.setPosition(0);
                axis2Switch = false;
            }

            packet.put("axisSwitch", axis2Switch);
            packet.put("wobbleAxis2Position", wobbleAxis2.getPosition());
            packet.put("cur1.b", cur1.b);
            packet.put("prev1.b", prev1.b);
            packet.put("loops", loops);
            dashboard.sendTelemetryPacket(packet);

            // intake
            /*if(gamepad2.right_bumper) {
                intake.setPower(1);
            } else if(gamepad2.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // shooter switch
            if(gamepad2.y && !shooterSwitch && !prev2.y) {
                shooter.setPower(1);
                shooterSwitch = true;
            } else if(gamepad2.y && shooterSwitch && !prev2.y) {
                shooter.setPower(0);
                shooterSwitch = false;
            }*/

            // drive logic
            /*double flPower;
            double frPower;
            double rlPower;
            double rrPower;

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            flPower = Range.clip(drive + strafe + rotate, -1.0, 1.0);
            frPower = Range.clip(drive - strafe - rotate, -1.0, 1.0);
            rlPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);
            rrPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);

            fl.setPower(flPower);
            fr.setPower(frPower);
            rl.setPower(rlPower);
            rr.setPower(rrPower);*/

            // telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            /*telemetry.addData("Motors", "flPower (%.2f), frPower (%.2f)", flPower, frPower);
            telemetry.addData("Motors", "rlPower (%.2f), rrPower (%.2f)", rlPower, rrPower);*/
            telemetry.update();

            // previous buttons pressed (for toggles)
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                idle();
            }
            loops++;
        }

    }
}
