package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by ftcuser2 on 10/29/16.
 */

public class AutonomousOp extends AutoDriveOp {

    private LightSensor front;
    private ColorSensor center, cs;
    private CRServo crservo;
    private boolean redTeam;

    public void runOpMode() throws InterruptedException {
        front=hardwareMap.lightSensor.get("front");
        center=hardwareMap.colorSensor.get("center");
        cs=hardwareMap.colorSensor.get("color");
        crservo=hardwareMap.crservo.get("servo");
        redTeam=true;
        initHardware();
        waitForStart();
        front.enableLed(true);
        center.enableLed(true);
        int initColor=center.red();
        double initLightVal=front.getRawLightDetected();

        moveInches(95); //move to roughly the white line

        //move so center is over white line
        while(center.red()-initColor<50){
            int leftPos = left.getCurrentPosition();
            int rightPos = right.getCurrentPosition();
            if (leftPos > rightPos) {
                left.setPower(.8 * (1 - Math.min(100, leftPos - rightPos) / 100d));
                right.setPower(.8);
            } else if (rightPos > leftPos) {
                right.setPower(.8 * (1 - Math.min(100, rightPos - leftPos) / 100d));
                left.setPower(.8);
            }
        }
        left.setPower(0);
        right.setPower(0);
        Thread.sleep(100);

        //align robot along the white line
//        while(front.getRawLightDetected()-initLightVal<50){
//            right.setPower(.25);
//            left.setPower(-.25);
//        }
//
//        left.setPower(0);
//        right.setPower(0);
//        Thread.sleep(100);

        //move towards beacon after alignment to allow scanning of beacon
//        moveInches(5);

//        cs.enableLed(true);
//        sleep(500);
//        left.setPower(0.1); right.setPower(0.1);
//        //rotate(-90 - getDirection());
//
//        //drive forward until colors
//        while(avg() < 1) {
//            sleep(30);
//            telemetry.addData("Color", avg());
//            telemetry.update();
//        }
//
//        //stop before the wall
//        sleep(100);
//        left.setPower(0); right.setPower(0);
//
//        //scan
//        scanBeacon();
//
//        //pressing the button
//        right.setPower(0.1); left.setPower(0.1);
//        sleep(1000);
//        right.setPower(-0.1); left.setPower(-0.1);
//        sleep(3000);
//        right.setPower(0); left.setPower(0);


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
