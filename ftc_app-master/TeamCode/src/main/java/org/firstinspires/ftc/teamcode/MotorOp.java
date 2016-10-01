package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Me on 9/24/16.
 */

@Autonomous(name="MotorOp", group="MotorTest")
public class MotorOp extends OpMode {
    private DcMotor motor;

    public void init(){
        motor=hardwareMap.dcMotor.get("Motor A");
    }

    public void loop(){
        motor.setPower(.2
        );
    }
}
