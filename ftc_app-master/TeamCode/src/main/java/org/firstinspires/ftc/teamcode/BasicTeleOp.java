package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends DriveOp {
	private boolean stickDrive;
    private boolean bDown;
    private DcMotor arm;
    private CRServo crservo;
    private Servo forkDrop;

    public void init() {
		super.init();
        stickDrive = true;
        arm = hardwareMap.dcMotor.get("arm");
        crservo = hardwareMap.crservo.get("servo");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forkDrop = hardwareMap.servo.get("forkDrop");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (arm.isBusy());
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
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

        if (gamepad1.x) {
            forkDrop.setPosition(1);
        }

        if (gamepad1.dpad_left) {
            crservo.setPower(.3);
        } else if (gamepad1.dpad_right) {
            crservo.setPower(-.3);
        } else {
            crservo.setPower(0);
        }

        if (gamepad1.y) {
            arm.setPower(-.3);
        } else if (gamepad1.a) {
            arm.setPower(1);
        } else {
            arm.setPower(0);
        }
        telemetry.addData("driveMode", (stickDrive ? "stick drive" : "trigger drive"));
        telemetry.addData("arm position", arm.getCurrentPosition());

    }
}
