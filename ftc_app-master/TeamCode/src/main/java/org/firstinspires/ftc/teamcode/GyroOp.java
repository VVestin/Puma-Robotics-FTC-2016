package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@Autonomous(name="GyroOp", group="SensorTest")
public class GyroOp extends OpMode {

    private GyroSensor gyro;
    private DcMotor left;
    private DcMotor right;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public void loop() {
        telemetry.addData("gyro: ", gyro.getHeading());
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);
    }

}
