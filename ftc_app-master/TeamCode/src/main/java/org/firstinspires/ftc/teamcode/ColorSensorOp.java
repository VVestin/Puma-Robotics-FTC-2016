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

public class ColorSensorOp extends LinearOpMode {

    private ColorSensor cs;
    private CRServo crservo;
    public void runOpMode() throws InterruptedException {
        int button = 0;
        cs = hardwareMap.colorSensor.get("color");
        crservo = hardwareMap.crservo.get("servo");
        waitForStart();
        ArrayList<LightReading> lightValues = new ArrayList<LightReading>();
        long start = System.currentTimeMillis();
        Color c = new Color();
        c.red(cs.red());
        c.green(cs.green());
        c.blue(cs.blue());
        int t = (int)(System.currentTimeMillis() - start);
        lightValues.add(new LightReading(t, avg(c), c));
        //for loop until sense black, until not black, until zero
        //assumption that black is always 0, 0, 0
        crservo.setPower(0.4);

        //scanning
        while(!isBlack()){
            crservo.setPower(0.4);
        }
        while(isBlack()){
            crservo.setPower(0.4);
        }
        while(!isBlack()){
            crservo.setPower(0.4);
        }
        while(isBlack()){
            crservo.setPower(-0.4);
        }

        //getting array values - identifying buttons
        int count = 0; int t1 = 0; int t2 = 0;
        while(count < 3){
            t = (int)(System.currentTimeMillis() - start);
            LightReading a = new LightReading (t, avg(c), c);
            lightValues.add(a);
            telemetry.addData("red", c.red(cs.red()));
            telemetry.addData("green", c.green(cs.green()));
            telemetry.addData("blue", c.blue(cs.blue()));
            telemetry.update();
            if (isBlack(a.colorValue) && !isBlack(lightValues.get((lightValues.size() - 1)).colorValue)){
                if (count == 1){
                    t1 = t; count++;
                }
                else if (count == 2){
                    t2 = t; count++;
                }
            }
            telemetry.addData("Button 1", t1);
            telemetry.addData("Button 2", t2);
        }

    }
    public double avg(Color a){
        return ((a.red(cs.red()) + a.green(cs.green()) + a.blue(cs.blue())) / 3.);
    }

    public boolean isBlack(){
        Color c = new Color();
        c.red(cs.red());
        c.green(cs.green());
        c.blue(cs.blue());
        return isBlack(c);
    }

    public boolean isBlack(Color c) {
        if(avg(c) == 0){
            return true;
        } else {
            return false;
        }
    }

    private class LightReading{
        //time = ms thing, light value = avg brightness, color Value = c, diff (i) LV - (i-1) LV
        private int time; private double lightValue; private Color colorValue;
        //constructor
        public LightReading(int time, double lightValue, Color colorValue){
            //instance variables
            this.time = time; this.lightValue = lightValue; this.colorValue = colorValue;
        }

    }

    //sys print is now telemetry.add data or telemetry.logdata
    //collections.sort

}
