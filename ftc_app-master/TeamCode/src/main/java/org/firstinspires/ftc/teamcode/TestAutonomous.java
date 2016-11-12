package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@Autonomous(name="AutonomousTest")
public class TestAutonomous extends AutoDriveOp {

    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        rotate(90);
        Thread.sleep(1000);
        rotate(90);
        Thread.sleep(1000);
        rotate(90);
        Thread.sleep(1000);
        rotate(90);
        Thread.sleep(1000);
        //rotate(90);

       /* rotateTo(-90);
        moveTicks(-1440);
        rotate(135);
        moveInches(6);
        rotate(180);*/
    }

}
