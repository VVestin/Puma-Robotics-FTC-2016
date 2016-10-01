package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by ftcuser1 on 10/1/16.
 * All angles in degrees.
 * All distances in inches.
 */

public abstract class AutoDriveOp extends LinearOpMode {
    // Diameter = 3.875 inches, Gear ratio = 1.5, 1440 ticks per rotation.
    protected static final double TICKS_PER_INCH = 1440 / (3.875 * Math.PI * 1.5); // TODO callibrate by testing

    protected DcMotor left, right;
    protected GyroSensor gyro;


    public abstract void runOpMode() throws InterruptedException;

    protected void initHardware() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        gyro = hardwareMap.gyroSensor.get("gyro");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        gyro.calibrate();
        while (gyro.isCalibrating());
    }

    protected void resetEncoders() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (right.isBusy() || left.isBusy());
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // moves a set number of encoder ticks
    protected void moveTicks(int ticks) {
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(right.getCurrentPosition() + ticks);
        left.setTargetPosition(left.getCurrentPosition() + ticks);
        while (right.isBusy() || left.isBusy());
    }

    protected void moveInches(double inches) {
        moveTicks((int) (inches * TICKS_PER_INCH));
    }

    // Takes gyro reading, but changes it to be between -180 and 180.
    protected int getDirection() {
        int angle = gyro.getHeading();
        if (angle > 180) angle -= 360;
        return -angle;
    }

    protected void rotateTo(int angle) {

    }

    protected void rotate(int angle) {

    }

}
