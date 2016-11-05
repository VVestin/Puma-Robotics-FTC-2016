package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by ftcuser2 on 10/29/16.
 */
@Autonomous(name="Autonomous")
public class AutonomousOp extends AutoDriveOp {

    private OpticalDistanceSensor ods;
    private ColorSensor cs;
    private ColorSensor front;
    private CRServo crservo;
    private boolean redTeam;

    public void runOpMode() throws InterruptedException {
        ods=hardwareMap.opticalDistanceSensor.get("ods");
        cs=hardwareMap.colorSensor.get("color");
        front = hardwareMap.colorSensor.get("front");
        crservo=hardwareMap.crservo.get("servo");
        redTeam=true;
        initHardware();
        waitForStart();
        ods.enableLed(true);
        double initLightVal = ods.getLightDetected();
        double initColorVal = avg(front);
        double power=.25;

        moveInches(35); //move to roughly the white line

        //move so center is over white line
        while(ods.getLightDetected()-initLightVal<.07){
            int leftPos = left.getCurrentPosition();
            int rightPos = right.getCurrentPosition();
            if (leftPos > rightPos) {
                left.setPower(power * (1 - Math.min(100, leftPos - rightPos) / 100d));
                right.setPower(power);
            } else if (rightPos > leftPos) {
                right.setPower(power * (1 - Math.min(100, rightPos - leftPos) / 100d));
                left.setPower(power);
            }
            if(avg(front)>10){
                power=.2;
                telemetry.addData("slowing down", true);
                telemetry.update();
            }
        }


        left.setPower(0);
        right.setPower(0);
        Thread.sleep(200);

        //align robot along the white line
        if (redTeam) {
            right.setPower(.1);
            left.setPower(-.1);
        } else {
            right.setPower(-.1);
            left.setPower(.1);
        }
        while(avg(front) < 10) {}

        left.setPower(0);
        right.setPower(0);

        cs.enableLed(true);
        sleep(500);
        left.setPower(0.1);
        right.setPower(0.1);

        //drive forward until colors
        while(avg(cs) < 1) {
            sleep(30);
            telemetry.addData("Color", avg(cs));
            telemetry.update();
        }

        //stop before the wall
        sleep(100);
        left.setPower(0); right.setPower(0);

        //scan
        scanBeacon();

        //pressing the button
        right.setPower(0.1); left.setPower(0.1);
        sleep(1000);
        right.setPower(-0.1); left.setPower(-0.1);
        sleep(3000);
        right.setPower(0); left.setPower(0);


    }

    public double avg() {
        return avg(cs);
    }

    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }


    public void scanBeacon(){
        try {
            //set passive
            cs.enableLed(false);
            sleep(500);
            double power = 0.3;
            crservo.setPower(power);
            //scan until correct or incorrect color
            while (Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))
                sleep(30);
            //turn if wrong color, else stay
            if (cs.red() > cs.blue() != redTeam) {
                power = -power;
                crservo.setPower(power);
                sleep(400);
                // Small sleep so that it goes back into the center
                while (Math.abs(cs.red() - cs.blue()) <= 1)
                    sleep(30); // Go until you see one color more than the other
                telemetry.addData("Reverse worked", cs.red() > cs.blue() == redTeam);
            }
            telemetry.addData("red", cs.red());
            telemetry.addData("blue", cs.blue());
            telemetry.update();
            //error in blue edge
            if (!redTeam)
                sleep(200);
            crservo.setPower(0);
            sleep(500);

            //button identification
            cs.enableLed(true);
            sleep(500);
            power /= 2;
            crservo.setPower(power);
            if (redTeam) {
                while (avg() > 5) sleep(35);
            } else {
                while (avg() > 5) {
                    sleep(35);
                }
            }
            crservo.setPower(0);
            telemetry.addData("done", true);
            telemetry.update();
        }
        catch(InterruptedException e) {
            e.printStackTrace();
        }
    }
}
