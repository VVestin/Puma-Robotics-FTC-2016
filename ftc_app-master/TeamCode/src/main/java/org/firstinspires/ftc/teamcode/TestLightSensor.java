package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by sarabranham on 10/29/16.
 */
@Autonomous(name = "TestLightSensor", group = "SensorTest")
public class TestLightSensor extends OpMode {
    private OpticalDistanceSensor ls;
    private OpticalDistanceSensor cs;

    public void init() {
        ls = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.opticalDistanceSensor.get("front");
    }

    public void loop(){
        ls.enableLed(true);
        cs.enableLed(true);
        telemetry.addData("Light Sensor: ", ls.getRawLightDetected());
        telemetry.addData("CS Light Detected", cs.getRawLightDetected());
    }


}
