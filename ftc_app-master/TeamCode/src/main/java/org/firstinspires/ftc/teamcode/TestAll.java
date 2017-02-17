package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by sarabranham on 12/10/16.
 */

public class TestAll extends ButtonPusher{
    private DcMotor arm1;
    private DcMotor arm2;
    private Servo forkDrop;

    public void init(){
        super.init();
        cs.enableLed(false);
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        forkDrop = hardwareMap.servo.get("forkDrop");
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        // Check Motors
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);

        // Check servo
        if (gamepad1.dpad_left) {
            crservo.setPower(.3);
        } else if (gamepad1.dpad_right) {
            crservo.setPower(-.3);
        } else {
            crservo.setPower(0);
        }

        // Check Arm
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

        // CS LED
        if (gamepad1.b) {
            cs.enableLed(true);
        } else if (gamepad1.x) {
            cs.enableLed(false);
        }

        // Check Fork Drop
        /*if (gamepad1.dpad_up) {
            forkDrop.setPosition(1);
        } else if (gamepad1.dpad_down) {
            forkDrop.setPosition(0);
        } else {
            forkDrop.setPosition(forkDrop.getPosition());
        }*/

        telemetry.addData("ods: ", ods.getLightDetected());
        telemetry.addData("front: ", front.getLightDetected());
        telemetry.addData("cs: ", avg(cs));
        telemetry.addData("forkDrop: ", forkDrop.getPosition());


    }

}
