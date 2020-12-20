package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="AutonomousTemplate", group="autonomous")
@Disabled
public class AutonomousTemplate extends LinearOpMode{

    // Variables
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    private BNO055IMU imu;

    @Override
    public void runOpMode() {

        // Configure Hardware
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        rl = hardwareMap.get(DcMotor.class, "back_left_motor");
        rr = hardwareMap.get(DcMotor.class, "back_right_motor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        fl.setDirection(DcMotor.Direction.REVERSE);
        rl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addData("Status", "Operational");
        telemetry.update();

        // Beginning of Autonomous

    }

    // Configures Drive Commands
    void Stop() {
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
    void Drive(float power, int ticks) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setTargetPosition(ticks);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setTargetPosition(ticks);
        rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setTargetPosition(ticks);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setTargetPosition(ticks);
        fl.setPower(power);
        fr.setPower(power);
        rl.setPower(power);
        rr.setPower(power);
        while(opModeIsActive()) {
            if(fl.getCurrentPosition() >= ticks) {
                break;
            }
        }
    }
    void Strafe(float power, int ticks) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setTargetPosition(ticks);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setTargetPosition(ticks);
        rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl.setTargetPosition(ticks);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setTargetPosition(ticks);
        fl.setPower(power);
        fr.setPower(power);
        rl.setPower(-power);
        rr.setPower(-power);
        while(opModeIsActive()) {
            if(fl.getCurrentPosition() >= ticks) {
                break;
            }
        }
    }
    void Rotate(float power, int value) {
        Orientation orientation;
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float target = orientation.firstAngle+value;
        if (value > 0) {
            fl.setPower(-power);
            fr.setPower(power);
            rl.setPower(-power);
            rr.setPower(power);
            while(opModeIsActive()) {
                Orientation current;
                current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (current.firstAngle >= target) {
                    break;
                }
            }
        } else {
            fl.setPower(power);
            fr.setPower(-power);
            rl.setPower(power);
            rr.setPower(-power);
            while(opModeIsActive()) {
                Orientation current;
                current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (current.firstAngle <= target) {
                    break;
                }
            }
        }
    }
}
