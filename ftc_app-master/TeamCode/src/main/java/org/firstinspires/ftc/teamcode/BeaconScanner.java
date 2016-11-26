package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by Sara Branham on 9/30/16.
 */

@Autonomous(name="BeaconScanner", group="SensorTest")

public class BeaconScanner extends LinearOpMode {

    private ColorSensor cs; private CRServo crservo;
    public void runOpMode() throws InterruptedException {
        cs = hardwareMap.colorSensor.get("color");
        cs.enableLed(false);
        crservo = hardwareMap.crservo.get("servo");
        waitForStart();
        boolean redTeam = false;
        while (true) {
            if (gamepad1.b) {
                redTeam = true;
                break;
            }
            if (gamepad1.x) {
                redTeam = false;
                break;
            }
        }
        double power = 0.3;
        crservo.setPower(power);
        while (Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))
            sleep(30); // Go until you see one color more than the other
        if (cs.red() > cs.blue() != redTeam) {
            power = -power;
            crservo.setPower(power);
            sleep(400); // Small sleep so that it goes back into the center
            while (Math.abs(cs.red() - cs.blue()) <= 1)
                sleep(30); // Go until you see one color more than the other
            telemetry.addData("Reverse worked", cs.red() > cs.blue() == redTeam);
        }
        telemetry.addData("red", cs.red());
        telemetry.addData("blue", cs.blue());
        telemetry.update();
        if (!redTeam)
            sleep(250);
        crservo.setPower(0);
        sleep(500);

        // Button identification
        cs.enableLed(true);
        sleep(500);
        power /= 2;
        crservo.setPower(power);
        if (redTeam) {
            while (avg() > 4) sleep(35);
        } else {
            while (avg() > 3.5) {
                sleep(35);
            }
        }
        crservo.setPower(0);
        telemetry.addData("done", true);
        telemetry.update();
        sleep(5000);
    }

    public double avg() {
        return (cs.red() + cs.green() + cs.blue()) / 3.0;
    }

    public boolean isBlack() {
        return avg() < 2.5;
    }

    private class LightReading{
        // Time = ms thing, light value = avg brightness, color Value = c, diff (i) LV - (i-1) LV
        private int time; private double lightValue; private Color colorValue;
        // Constructor
        public LightReading(int time, double lightValue, Color colorValue){
            // Instance variables
            this.time = time; this.lightValue = lightValue; this.colorValue = colorValue;
        }

    }


}
