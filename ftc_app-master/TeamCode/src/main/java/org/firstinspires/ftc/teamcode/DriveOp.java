package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

public abstract class DriveOp extends OpMode {
	protected static final double TICKS_PER_INCH = 1440 / (3.875 * Math.PI * 1.5); // TODO callibrate by testing

    protected DcMotor left, right;
    //protected GyroSensor gyro;

	public void init() {
        telemetry.addData("Initializing DriveOp", true);

		left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //gyro = hardwareMap.gyroSensor.get("gyro");

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        //gyro.calibrate();
	}

    protected int getDirection() {
        int angle = 0;//gyro.getHeading();
        if (angle > 180) angle -= 360;
        return -angle;
    }

    protected void resetEncoders() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (right.isBusy() || left.isBusy());
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
