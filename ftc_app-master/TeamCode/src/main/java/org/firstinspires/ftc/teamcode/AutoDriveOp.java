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

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    // Moves a set number of encoder ticks
    protected void moveTicks(int ticks) {
//        telemetry.addData("Moving ticks", ticks);
//        telemetry.update();
        resetEncoders();
        double maxPower = 0.5; double minPower = 0.2;
        right.setPower(maxPower);
        left.setPower(maxPower);

        while (right.getCurrentPosition() < ticks || left.getCurrentPosition() < ticks){
            double power = minPower+(maxPower - minPower)*(((double)ticks - left.getCurrentPosition())/(double)ticks);
            right.setPower(power); left.setPower(power);
        }

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

    protected void rotate(int angle){
        gyro.resetZAxisIntegrator();
        int leftDir = 1;
        int rightDir = 1;
        if (angle > 0) {
            rightDir = -1;
        } else {
            leftDir = -1;
        }
        double maxPower = 0.3;
        double minPower = 0.1;
        while(Math.abs(getDirection()) < Math.abs(angle)){
            double power = minPower + (maxPower - minPower)*((Math.abs(angle) - Math.abs(getDirection())) / ((double) Math.abs(angle)));
            left.setPower(power * leftDir);
            right.setPower(power * rightDir);
            telemetry.addData("Power", power);
            telemetry.addData("Direction", getDirection());
            telemetry.update();
        }
        left.setPower(0); right.setPower(0);
    }

    // Rotate relative to current position (- is left, + is right)
    // We want to try doing this with encoders
    // FFS comment your damn code Weston
    protected void rotateBad(int angle) {
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
//                left.setPower(power * (1 - Math.min(100, leftPos - rightPos) / 100d) * leftDir);
                left.setPower(power*leftDir);
                right.setPower(power * rightDir);
            } else if (rightPos > leftPos) {
//                right.setPower(power * (1 - Math.min(100, rightPos - leftPos) / 100d) * rightDir);
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

    protected void turn(int angle) {
        int initHeading=getDirection();
        int heading=getDirection();
    }

}
