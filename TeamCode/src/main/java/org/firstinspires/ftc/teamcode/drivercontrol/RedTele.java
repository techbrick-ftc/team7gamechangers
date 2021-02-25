// Our primary teleop program, utilizing FieldCentric.java

package org.firstinspires.ftc.teamcode.drivercontrol;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.zimportants.AutoImport;

@TeleOp(name="RedTele", group="Main")
public class RedTele extends AutoImport{

    public RedTele() {
        super(0, 0, 0, 0);
    }

    FieldCentric drive = new FieldCentric();

    private int shooterTPS = -1540;

    // Tells auto class if the abort button (for automatically shooting rings) had been pressed
    public boolean driverAbort() {
        return gamepad1.y;
    }

    public void runOpMode() {
        super.runOpMode();

        int loops = 0;
        ElapsedTime launchServoTime = null;

        // adds start telemetry
        telemetry.addLine("hardware ready");
        telemetry.update();

        // Defines motor configs
        final double PI = Math.PI;
        DcMotor[] motors = {m1, m2, m3, m4};
        double[] motorAngles = {3*PI/4, 5*PI/4, 7*PI/4, PI/4};

        // Sets up motor configs
        try {
            drive.setUp(motors, motorAngles, imu);
        } catch (Exception e) {
            packet.put("SETUP ERROR", true);
            dashboard.sendTelemetryPacket(packet);
            e.printStackTrace();
        }

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Configures Variables
        boolean axis2Switch = false;
        boolean intakeSwitch = false;

        // Things that move in init
        ringLock.setPosition(1);
        
        waitForStart();

        // Robot Control
        while(opModeIsActive()) {
            // Updates cur1 & 2
            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            packet.put("cur2", cur2.x);
            packet.put("trigger", gamepad2.right_trigger);
            dashboard.sendTelemetryPacket(packet);

            // Set wheel powers
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // Wobble Grabber Control
            double wobbleY = gamepad2.left_stick_y;
            double wobble1Power = Range.clip(wobbleY, -0.75, 0.75);
            if (armTouch.isPressed()) {
                wobble1Power = Range.clip(wobbleY, -0.75, 0);
            }
            wobbleAxis1.setPower(wobble1Power);

            if (cur2.b && !prev2.b && !axis2Switch) {
                wobbleAxis2.setPosition(0.5);
                axis2Switch = true;
            } else if (cur2.b && !prev2.b && axis2Switch) {
                wobbleAxis2.setPosition(0);
                axis2Switch = false;
            }

            // Intake Control
            if (cur2.right_bumper && !prev2.right_bumper && !intakeSwitch) {
                ringLock.setPosition(0.95);
                intake1.setPower(1);
                intakeSwitch = true;
            } else if (cur2.right_bumper && !prev2.right_bumper && intakeSwitch) {
                ringLock.setPosition(0.95);
                intake1.setPower(0);
                intakeSwitch = false;
            } else if (gamepad2.left_bumper) {
                ringLock.setPosition(-1);
                intake1.setPower(-1);
                intakeSwitch = true;
            }

            // Shooter Control
            if (gamepad2.right_trigger > 0.1) {
                shooter.setVelocity(shooterTPS);
            } else {
                shooter.setVelocity(0);
            }

            // Shooter Servo Control
            if (launchServoTime == null && cur2.a && !prev2.a) {
                launchServoTime = new ElapsedTime();
                ringLock.setPosition(0.4);
                shooterServo.setPosition(0.3);
            } else if (launchServoTime != null && launchServoTime.milliseconds() > 600) {
                launchServoTime = null;
            } else if (launchServoTime != null && launchServoTime.milliseconds() > 300) {
                ringLock.setPosition(0.95);
                shooterServo.setPosition(1);
            }

            // Tape Measure Control
            if (cur2.x) {
                tapeMeasure.setPower(1);
            } else if (cur2.y) {
                tapeMeasure.setPower(-1);
            } else {
                tapeMeasure.setPower(0);
            }

            // Drive to High Shots
            if (cur1.x) {
                shooter.setVelocity(-1500);
                slauto.drive(2, 39, 0, 1, this);
                if (!this.driverAbort()) { shoot(-1500, 3, 0, 500, true); }
            }

            // Drive to Power Shots
            if (cur1.b) {
                // spins up flywheel
                shooter.setVelocity(-1350); // orig -1350

                // drives to first power shot and shoots
                slauto.drive(-4, 23, 0, 1, this);
                if (!this.driverAbort()) {
                    shoot(-1350, 1, 0, 100, false);

                    // drives to second power shot and shoots
                    slauto.drive(-4, 17, 0, 1, this);
                    shoot(-1310, 1, 0, 100, false);

                    // drives to third power shot and shoots
                    slauto.drive(-4, 9, 0, 1, this);
                    shoot(-1310, 1, 0, 100, true);
                }
            }

            // Reset Field Centric button
            if (cur1.a && !prev1.a) {
                drive.newOffset();
            }

            // High shot button
            if (cur2.dpad_up && !prev2.dpad_up) {
                shooterTPS = -1540;
            }

            // Power shot Button
            if (cur2.dpad_down && !prev2.dpad_down) {
                shooterTPS = -1390;
            }

            // Updates prev1 & 2
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Send FTC Dashboard Packets
            packet.put("loop count", loops);
            packet.put("fr power", m1.getPower());
            packet.put("fl power", m4.getPower());
            packet.put("rr power", m2.getPower());
            packet.put("rl power", m3.getPower());
            packet.put("arm sensor", armTouch.isPressed());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("wobble encoder", wobbleAxis1.getCurrentPosition());
            telemetry.addData("wobble servo position", wobbleAxis2.getPosition());
            telemetry.update();

            loops++;
        }

        /*slamra.stop();*/
    }
}
