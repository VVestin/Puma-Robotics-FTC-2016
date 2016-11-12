package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by sarabranham on 11/12/16.
 */

@Autonomous(name="SecondBeacon")
public class SecondBeacon extends AutoDriveOp{

    private OpticalDistanceSensor ods;
    private ColorSensor cs;
    private ColorSensor front;
    private CRServo crservo;
    private boolean redTeam;

    public void runOpMode() throws InterruptedException{
        ods=hardwareMap.opticalDistanceSensor.get("ods");
        cs=hardwareMap.colorSensor.get("color");
        front = hardwareMap.colorSensor.get("front");
        crservo=hardwareMap.crservo.get("servo");
        redTeam=false;
        initHardware();
        waitForStart();
        ods.enableLed(true);
        double initLightVal = ods.getLightDetected();
        double initColorVal = avg(front);
        double power=.25;

        //ends after driving backwards (assume end of white line) - both motors stopped

        //turn left
        right.setPower(0.1); left.setPower(-0.1);
        sleep(500);
        right.setPower(0); left.setPower(0);

        //drive straight until see white
        while(avg() < 10){
            right.setPower(0.1);
            left.setPower(0.1);
        }

    }

    public double avg() {
        return avg(cs);
    }
    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }
}
