package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by sarabranham on 1/21/17.
 */

@Autonomous(name="ScannerTest")

public class BeaconTest extends ButtonPusher {

    public void init() {
        super.init();
        state = State.SCAN_BEACON;
    }

    public void loop() {
        super.loop();
    }



}
