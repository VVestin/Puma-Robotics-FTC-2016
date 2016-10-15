package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@Autonomous(name="ServoTest")
public class ServoTestOp extends LinearOpMode {
//    private

    public void runOpMode() throws InterruptedException {
        ServoController controller=hardwareMap.servoController.get("servos");
        CRServo s = hardwareMap.crservo.get("servo");
        telemetry.addData("init", true);
        waitForStart();
        telemetry.addData("start", true);
//        controller.pwmEnable();
//        telemetry.addData("status",controller.getPwmStatus());
        s.setDirection(DcMotorSimple.Direction.FORWARD);
        s.setPower(.7);
        telemetry.addData("hello","gfy");
        Thread.sleep(2500);

    }

}
