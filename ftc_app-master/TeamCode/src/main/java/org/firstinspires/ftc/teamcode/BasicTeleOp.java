package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends ButtonPusher implements BeaconConstants {
    protected static final double FORK_LOCKED = 1, FORK_UNLOCKED = 0;
    protected boolean bDown;
    protected boolean backwards = false;
    protected boolean xDown;
    protected boolean stopArm;
    protected boolean slowDrive;
    protected CRServo crservo;

    public void init() {
        super.init();
        stopArm = true;
        slowDrive = false;
        crservo = hardwareMap.crservo.get("servo");
        forkDrop = hardwareMap.servo.get("forkDrop");
        forkDrop.setPosition(FORK_UNLOCKED);
        state = State.DRIVER_CONTROL;
    }

    public void start() {
        forkDrop.setPosition(FORK_LOCKED);
        stopArm = true;
    }

    public void loop() {
        if (state == State.DRIVER_CONTROL) {
            telemetry.addData("left power:", left.getPower());
            telemetry.addData("right power:", right.getPower());
//            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("driver control", true);
            if (slowDrive && (gamepad1.left_stick_y/Math.abs(gamepad1.left_stick_y) == gamepad1.right_stick_y/Math.abs(gamepad1.right_stick_y))) {
                left.setPower(Math.pow(-gamepad1.left_stick_y, 3) * 0.6);
                right.setPower(Math.pow(-gamepad1.right_stick_y, 3) * 0.6);
            } else {
                left.setPower(Math.pow(-gamepad1.left_stick_y, 3));
                right.setPower(Math.pow(-gamepad1.right_stick_y, 3));
            }
            if (gamepad1.b && !bDown) {
                slowDrive = !slowDrive;
                bDown = true;
            } else if (!gamepad1.b) {
                bDown = false;
            }

            if (gamepad1.x && !xDown) {
                stopArm = !stopArm;
                if (stopArm)
                    forkDrop.setPosition(FORK_LOCKED);
                else
                    forkDrop.setPosition(FORK_UNLOCKED);
                xDown = true;
            } else if (!gamepad1.x) {
                xDown = false;
            }

            if (gamepad1.dpad_left) {
                crservo.setPower(CR_POWER);
            } else if (gamepad1.dpad_right) {
                crservo.setPower(-CR_POWER);
            } else {
                crservo.setPower(0);
            }

            if (gamepad1.right_bumper) {
                backwards = true;
            } else if (gamepad1.left_bumper) {
                backwards = false;
            }

            if (!stopArm) {
                if (gamepad1.y) {
                    arm1.setPower(ARM_POWER);
                    arm2.setPower(ARM_POWER);
                } else if (gamepad1.a) {
                    arm1.setPower(-ARM_SLOW_POWER);
                    arm2.setPower(-ARM_SLOW_POWER);
                } else {
                    arm1.setPower(0);
                    arm2.setPower(0);
                }
            } else {
                arm1.setPower(0);
                arm2.setPower(0);
            }
            telemetry.addData("arm position", arm1.getCurrentPosition());
        } else {
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            super.loop();
        }
    }
}
