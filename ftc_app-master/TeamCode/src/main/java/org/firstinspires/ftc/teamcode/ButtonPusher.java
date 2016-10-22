package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.BeaconScanner;

/**
 * Created by sarabranham on 10/22/16.
 */

@Autonomous(name="ButtonPusher", group="SensorTest")

public class ButtonPusher extends AutoDriveOp {
    private ColorSensor cs; private CRServo crservo; private boolean redTeam;
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        redTeam = true;
        cs = hardwareMap.colorSensor.get("color");
        crservo = hardwareMap.crservo.get("servo");
        cs.enableLed(true);
        sleep(500);
        left.setPower(0.1); right.setPower(0.1);
        //rotate(-90 - getDirection());

        //drive forward until colors
        while(avg() < 1) {
            sleep(30);
            telemetry.addData("Color", avg());
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
        return (cs.red() + cs.green() + cs.blue()) / 3.0;
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
