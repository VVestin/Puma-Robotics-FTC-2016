package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@Autonomous(name="ServoTest")
public class ServoTestOp extends LinearOpMode {
    private Servo s = hardwareMap.servo.get("servo");

    public void runOpMode() {
        while (true) {
            telemetry.addData("pos", s.getPosition());
            telemetry.addData("dir", s.getDirection());
        }
    }

}
