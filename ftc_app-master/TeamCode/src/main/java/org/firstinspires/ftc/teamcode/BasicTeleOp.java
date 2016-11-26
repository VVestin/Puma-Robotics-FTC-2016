package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import java.util.Stack;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends ButtonPusher {
	private boolean beacon;
	private boolean stickDrive;

    public void init() {
		super.init();
        stickDrive = true;
        beacon = false;
    }

    public void loop() {
        if (state == State.DRIVER_CONTROL) {
			telemetry.addData("driver control", true);
			telemetry.addData("drive mode", stickDrive ? "stick" : "trigger");
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
			
			if (gamepad1.dpad_left) {
                alignRight = false;
                state = State.PUSH_BEACON_START;
            } else if (gamepad1.dpad_right){
                alignRight = true;
                state = State.PUSH_BEACON_START;
            }
            telemetry.addData("driveMode", (stickDrive ? "stick drive" : "trigger drive"));
        } else {
			telemetry.addData("driver control", false);
			super.loop();
			if (!gamepad1.atRest()) {
				state = State.DRIVER_CONTROL;
			}
        }

    }
}
