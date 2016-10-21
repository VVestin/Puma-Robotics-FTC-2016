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
        telemetry.addData("Moving ticks", ticks);
        telemetry.update();
        resetEncoders();
        double power = 0.8;
        right.setPower(power);
        left.setPower(power);

        while (right.getCurrentPosition() < ticks || left.getCurrentPosition() < ticks) {
            int leftPos = left.getCurrentPosition();
            int rightPos = right.getCurrentPosition();
            if (leftPos > rightPos) {
                left.setPower(power * (1 - Math.min(100, leftPos - rightPos) / 100d));
                right.setPower(power);
            } else if (rightPos > leftPos) {
                right.setPower(power * (1 - Math.min(100, rightPos - leftPos) / 100d));
                left.setPower(power);
            }
            telemetry.addData("right", rightPos);
            telemetry.addData("left", leftPos);
            telemetry.addData("rightPow", right.getPower());
            telemetry.addData("leftPow", left.getPower());
            telemetry.update();
        }
        telemetry.addData("movedRight", right.getCurrentPosition());
        telemetry.addData("movedLeft", left.getCurrentPosition());
        telemetry.update();
        right.setPower(0);
        left.setPower(0);
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

    // Rotate to an absolute angle (0 degrees is start)
    protected void rotateTo(int angle) {
        int currentHeading=getDirection();
        while(currentHeading != angle){
            if(Math.abs(currentHeading-angle)==180 || Math.abs(currentHeading-angle)>180){
                left.setPower(.7);
                right.setPower(-.7);
            }else if(Math.abs(currentHeading-angle)<180){
                left.setPower(-.7);
                right.setPower(.7);
            }
            currentHeading=getDirection();
        }
    }

    // Rotate relative to current position (- is left, + is right)
    protected void rotate(int angle) {
        int initHeading=getDirection();
        int currentHeading=getDirection();
        while(currentHeading != angle){
            int relHeading=currentHeading-initHeading;
            if(Math.abs(relHeading-angle)==180 || Math.abs(relHeading-angle)<180){
                left.setPower(.3);
                right.setPower(-.3);
            }else if(Math.abs(relHeading-angle)>180){
                left.setPower(-.3);
                right.setPower(.3);
            }
            currentHeading=getDirection();
            telemetry.addData("angle",currentHeading);
            telemetry.update();
        }
    }

}
