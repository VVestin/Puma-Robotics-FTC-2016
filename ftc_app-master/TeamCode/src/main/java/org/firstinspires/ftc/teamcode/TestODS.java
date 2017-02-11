package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by ftcuser2 on 11/5/16.
 */

@Autonomous(name = "ODSTest", group = "SensorTest")
public class TestODS extends OpMode {

    private OpticalDistanceSensor ods;
    private OpticalDistanceSensor front;

    public void init() {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        front = hardwareMap.opticalDistanceSensor.get("front");
        ods.enableLed(true);
        front.enableLed(true);
    }

    public void loop() {
        telemetry.addData("Front: ", front.getRawLightDetected());
        telemetry.addData("ODS: ", ods.getRawLightDetected());
    }
}
