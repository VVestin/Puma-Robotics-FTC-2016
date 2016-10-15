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

        moveInches(142);
        //moveInches(-12);
        //rotate(180);

        /*rotate(45);
        rotateTo(-90);
        moveTicks(-1440);
        rotate(135);
        moveInches(6);
        rotate(180);*/
    }

}
