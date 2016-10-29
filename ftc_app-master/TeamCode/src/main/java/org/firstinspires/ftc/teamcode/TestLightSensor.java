package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by sarabranham on 10/29/16.
 */
@Autonomous(name = "TestLightSensor", group = "SensorTest")

public class TestLightSensor extends OpMode {
    private LightSensor ls;
    private ColorSensor cs;

    public void init() {
        ls = hardwareMap.lightSensor.get("light reading");
        cs = hardwareMap.colorSensor.get("color reading");
    }

    public void loop(){
        ls.enableLed(true);
        cs.enableLed(true);
        telemetry.addData("Light Sensor: ", ls.getRawLightDetected());
        telemetry.addData("CS Red: ", cs.red());
        telemetry.addData("CS Green: ", cs.green());
        telemetry.addData("CS Blue: ", cs.blue());
        telemetry.addData("CS Light Detected", cs.alpha());
    }


}
