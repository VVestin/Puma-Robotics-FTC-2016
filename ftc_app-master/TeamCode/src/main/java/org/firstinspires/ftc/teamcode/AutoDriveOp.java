package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by ftcuser1 on 10/1/16.
 * All angles in degrees.
 * All distances in inches.
 *
 * Motor 1 of the motor controller is left
 * Motor 2 of the motor controller is right
 * Left forward, right backwards = gyro +
 * Left backwards, right forwards = gyro -
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
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        gyro.calibrate();
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

    // Rotate relative to current position (- is left, + is right)
    // We want to try doing this with encoders
    // ffs comment your damn code weston
    protected void rotate(int angle) {
            int initHeading=getDirection();
            gyro.resetZAxisIntegrator();
            final double maxPower = 0.5;
            final double minPower = 0.2;
            double power = maxPower;
            resetEncoders();
            double leftDir, rightDir;
            if (angle > 0) {
                leftDir = 1;
                rightDir = -1;
            } else {
                rightDir = 1;
                leftDir = -1;
            }
            left.setPower(power * leftDir);
            right.setPower(power * rightDir);
            while (Math.abs(getDirection()-initHeading) < Math.abs(angle)) {
                try {
                    sleep(35);
                }
                catch (InterruptedException e){
                    e.printStackTrace();
                }
                power = maxPower * Math.min(maxPower, ((Math.min(90, Math.abs(gyro.getHeading() - angle))) / 90d) + minPower);
                int leftPos = Math.abs(left.getCurrentPosition());
                int rightPos = Math.abs(right.getCurrentPosition());
                if (leftPos > rightPos) {
//                    left.setPower(power * (1 - Math.min(100, leftPos - rightPos) / 100d) * leftDir);
                    left.setPower(power*leftDir);
                    right.setPower(power * rightDir);
                } else if (rightPos > leftPos) {
//                    right.setPower(power * (1 - Math.min(100, rightPos - leftPos) / 100d) * rightDir);
                    right.setPower(power * rightDir);
                    left.setPower(power * leftDir);
                }
                telemetry.addData("angle", Math.abs(getDirection()-initHeading));
                telemetry.addData("power", power);
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);

        }

    protected void turn(int angle){
        int initHeading=getDirection();
        int heading=getDirection();

    }

}
