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

    public void init() {
        s = hardwareMap.crservo.get("servo");
    }

    public void loop() {
        if (gamepad1.dpad_up) {
            s.setPower(.5);
        } else if (gamepad1.dpad_down) {
            s.setPower(-.5);
        } else {
            s.setPower(0);
        }
    }

}
