package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends OpMode {
    private DcMotor left;
    private DcMotor right;
    private boolean stickDrive;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        left.setDirection(DcMotor.Direction.REVERSE);
        right = hardwareMap.dcMotor.get("right");
        stickDrive = true;
    }

    public void loop() {
        if (stickDrive) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);
            if (gamepad1.b) {
                stickDrive = false;
            }
        } else {
            if (gamepad1.right_bumper) {
                right.setPower(-.5);
            } else {
                right.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_bumper) {
                left.setPower(-.5);
            } else {
                left.setPower(gamepad1.left_trigger);
            }
            if (gamepad1.a) {
                stickDrive = true;
            }
        }
        telemetry.addData("driveMode", (stickDrive ? "stick drive" : "trigger drive"));
    }

}
