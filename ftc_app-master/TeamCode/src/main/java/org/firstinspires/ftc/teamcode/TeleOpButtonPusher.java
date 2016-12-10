package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="TeleOp", group="MotorTest")
public class TeleOpButtonPusher extends ButtonPusher {
    private boolean stickDrive;
    private boolean pushingBeacon;
    private boolean bDown;
    private DcMotor arm1;
    private DcMotor arm2;
    private Servo forkDrop;

    public void init() {
        super.init();
        stickDrive = true;
        pushingBeacon = false;
        state=state.DRIVER_CONTROL;
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        forkDrop = hardwareMap.servo.get("forkDrop");
    }

//    public void start() {
//        forkDrop.setPosition(1);
//   }

    public void loop() {
        if (state == State.DRIVER_CONTROL) {
            if (!pushingBeacon) {
                telemetry.addData("driver control", true);
                telemetry.addData("drive mode", stickDrive ? "stick" : "trigger");
                if (stickDrive) {
                    left.setPower(-gamepad1.left_stick_y);
                    right.setPower(-gamepad1.right_stick_y);
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
                }

                if (gamepad1.b && !bDown) {
                    stickDrive = !stickDrive;
                    bDown = true;
                } else if (!gamepad1.b) {
                    bDown = false;
                }

//                if (gamepad1.x) {
//                    forkDrop.setPosition(1);
//                }

                if (gamepad1.dpad_left) {
                    crservo.setPower(.3);
                } else if (gamepad1.dpad_right) {
                    crservo.setPower(-.3);
                } else {
                    crservo.setPower(0);
                }

                if (gamepad1.y) {
                    arm1.setPower(-.3);
                    arm2.setPower(-.3);
                } else if (gamepad1.a) {
                    arm1.setPower(0.8);
                    arm2.setPower(0.8);
                } else {
                    arm1.setPower(0);
                    arm2.setPower(0);
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
            if (gamepad1.dpad_down) {
                state = State.DRIVER_CONTROL;
            }
        }

    }
}
