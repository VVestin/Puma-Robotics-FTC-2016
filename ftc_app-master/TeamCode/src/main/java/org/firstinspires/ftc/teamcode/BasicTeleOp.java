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
public class BasicTeleOp extends ButtonPusher {
    protected static final double FORK_LOCKED = 1, FORK_UNLOCKED = 0;
    protected boolean bDown;
    protected boolean xDown;
    protected boolean stopArm;
    protected boolean slowDrive;
    protected DcMotor arm1;
    protected DcMotor arm2;
    protected CRServo crservo;

    public void init() {
        super.init();
        stopArm = true;
        slowDrive = false;
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
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
            telemetry.addData("driver control", true);
            if (slowDrive) {
                left.setPower(Math.pow(-gamepad1.left_stick_y, 3) / 5d);
                right.setPower(Math.pow(-gamepad1.right_stick_y, 3) / 5d);
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
                crservo.setPower(.3);
            } else if (gamepad1.dpad_right) {
                crservo.setPower(-.3);
            } else {
                crservo.setPower(0);
            }

            if (!stopArm) {
                if (gamepad1.y) {
                    arm1.setPower(1);
                    arm2.setPower(1);
                } else if (gamepad1.a) {
                    arm1.setPower(-.25);
                    arm2.setPower(-.25);
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
            super.loop();
        }
    }
}
