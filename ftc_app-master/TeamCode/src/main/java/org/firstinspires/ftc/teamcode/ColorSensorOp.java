package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by ftcuser1 on 9/30/16.
 */

@Autonomous(name="ColorSensor", group="SensorTest")

public class ColorSensorOp extends OpMode {

    private ColorSensor color;

    public void init() {
        color = hardwareMap.colorSensor.get("color");
    }

    public void loop() {
        telemetry.addData("red", color.red());
        telemetry.addData("green", color.green());
        telemetry.addData("blue", color.blue());
        telemetry.addData("alpha", color.alpha());
    }

}
