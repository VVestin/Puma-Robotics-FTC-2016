package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by sarabranham on 10/22/16.
 */

@Autonomous(name="ButtonPusher", group="SensorTest")

public class ButtonPusher extends AutoDriveOp {
    private ColorSensor cs;
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        cs = hardwareMap.colorSensor.get("color");
        cs.enableLed(true);
        sleep(500);
        left.setPower(-0.1); right.setPower(-0.1);
        //rotate(-90 - getDirection());
        while(avg() < 4) {
            sleep(30);
            telemetry.addData("Color", avg());
            telemetry.update();
        }
        left.setPower(0); right.setPower(0);
    }

    public double avg() {
        return (cs.red() + cs.green() + cs.blue()) / 3.0;
    }
}
