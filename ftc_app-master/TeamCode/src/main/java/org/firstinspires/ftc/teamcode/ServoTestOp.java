package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp (name="ServoTest")
public class ServoTestOp extends OpMode {
    private CRServo s;
    private boolean hiSara;
    private long start;

    public void init() {
        s = hardwareMap.crservo.get("servo");
    }

    public void loop() {
        double power = 0.3;
        if(gamepad1.a){
            s.setPower(power);
            start = System.currentTimeMillis();

        } else if(gamepad1.b && !hiSara){
            s.setPower(0);
            hiSara = true;
            telemetry.addData("Time", System.currentTimeMillis() - start);
        }
    }
}
