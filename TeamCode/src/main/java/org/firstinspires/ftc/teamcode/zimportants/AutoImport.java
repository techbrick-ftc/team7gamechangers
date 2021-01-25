package org.firstinspires.ftc.teamcode.zimportants;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.zimportants.TeleAuto;

import static java.lang.Math.abs;

public class AutoImport {

    // Defines globally used variables
    private DcMotorEx shooter;
    private Servo loader;

    private double[] wheelPowers;

    // Instantiates the T265 camera
    private T265Camera slamra;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotorEx shooter, Servo loader) {
        this.shooter = shooter;
        this.loader = loader;
    }

    public void shoot(double tps, double amount, long rev, long delay) {
        shooter.setVelocity(tps);
        sleep(rev);
        for (double i = 0; i >= amount; i++) {
            shooter.setVelocity(tps);
            sleep(delay);
            loader.setPosition(0);
            sleep(1000);
            loader.setPosition(1);
        }
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

