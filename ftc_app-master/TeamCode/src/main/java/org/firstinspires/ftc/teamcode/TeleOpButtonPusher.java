package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="TeleOp", group="MotorTest")
public class TeleOpButtonPusher extends BasicTeleOp {

    private boolean pushingBeacon;

    public void init() {
        super.init();
        pushingBeacon = false;
        state=state.DRIVER_CONTROL;
//        forkDrop = hardwareMap.servo.get("forkDrop");
    }

//    public void start() {
//        forkDrop.setPosition(1);
//   }

    public void loop() {
        if (state == State.DRIVER_CONTROL) {
            if (!pushingBeacon) {
                super.loop();
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
        } else {
            telemetry.addData("driver control", false);
            super.loop();
            if (gamepad1.dpad_down) {
                state = State.DRIVER_CONTROL;
            }
        }
    }
}
