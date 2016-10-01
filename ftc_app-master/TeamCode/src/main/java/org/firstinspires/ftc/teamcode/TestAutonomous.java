package org.firstinspires.ftc.teamcode;

/**
 * Created by ftcuser1 on 10/1/16.
 */

public class TestAutonomous extends AutoDriveOp {

    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        moveTicks(7200); // 5 rotations of the motor
        rotate(45);
        rotateTo(-90);
        moveTicks(-1440); // 1 rotation of the motor
    }

}
