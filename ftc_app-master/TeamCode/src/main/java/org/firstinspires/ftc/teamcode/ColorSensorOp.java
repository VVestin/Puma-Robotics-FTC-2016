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

@Autonomous(name="ColorSensor", group="SensorTest")

public class ColorSensorOp extends OpMode {

    private ColorSensor cs;
    private CRServo s;

    // Start in passive - identify color (start with finding blue)
    // Once color, move to active and find button (take averages or contrast?)

    public void init() {
        cs = hardwareMap.colorSensor.get("color");
        s = hardwareMap.crservo.get("servo");

    }
    public void loop() {
        if (gamepad1.dpad_up) {
            s.setPower(.2);
        } else if (gamepad1.dpad_down) {
            s.setPower(-.2);
        } else {
            s.setPower(0);
        }
        if (gamepad1.a){
            cs.enableLed(true);
        } else if (gamepad1.b){
            cs.enableLed(false);
        }
        telemetry.addData("red", cs.red());
        telemetry.addData("green", cs.green());
        telemetry.addData("blue", cs.blue());
        telemetry.update();
    }
}