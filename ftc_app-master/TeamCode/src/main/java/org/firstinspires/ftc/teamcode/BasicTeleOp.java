package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends ButtonPusher {
	private boolean stickDrive;
    private boolean pushingBeacon;

    public void init() {
		super.init();
        stickDrive = true;
        pushingBeacon = false;
        state=state.DRIVER_CONTROL;
    }

    public void loop() {
        if (state == State.DRIVER_CONTROL) {
            if (!pushingBeacon) {
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
                    crservo.setPower(.3);
                } else if (gamepad1.dpad_right) {
                    crservo.setPower(-.3);
                } else {
                    crservo.setPower(0);
                }

                if (gamepad1.dpad_up) {
                    pushingBeacon = true;
                }
            } else {
                if (gamepad1.dpad_left) {
                    alignRight = false;
                    state = State.PUSH_BEACON_START;
                    nextStates.push(State.DRIVER_CONTROL);
                    pushingBeacon = false;
                } else if (gamepad1.dpad_right) {
                    alignRight = true;
                    state = State.PUSH_BEACON_START;
                    nextStates.push(State.DRIVER_CONTROL);
                    pushingBeacon = false;
                }
            }
            telemetry.addData("driveMode", (stickDrive ? "stick drive" : "trigger drive"));
        } else {
			telemetry.addData("driver control", false);
			super.loop();
			if (gamepad1.x) {
				state = State.DRIVER_CONTROL;
			}
        }

    }
}
