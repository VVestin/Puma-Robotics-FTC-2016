package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Integer.parseInt;

/**
 * Created by ftcuser2 on 10/29/16.
 */
@Autonomous(name="Autonomous")
public class AutonomousOp extends AutoDriveOp implements BeaconConstants{

    private OpticalDistanceSensor ods;
    private ColorSensor cs;
    private ColorSensor front;
    private CRServo crservo;
    private double crPower = CR_POWER;

    //Vuforia stuff
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable wheels;
    private VuforiaTrackable tools;
    private VuforiaTrackable legos;
    private VuforiaTrackable gears;
    private VuforiaTrackableDefaultListener wheelListener;
    private VuforiaTrackableDefaultListener toolListener;
    private VuforiaTrackableDefaultListener legoListener;
    private VuforiaTrackableDefaultListener gearListener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    //stupidly long license key
    public static final String VUFORIA_KEY = "AV4ONxv/////AAAAGefaDmLKjkgWifNHOt4h8QgFb23EhhiUz7Po/rcnXDMuJHa2Okvh/NLUSza5phLaIuyvWUINyu/cyKpmChyUCJ/A05QHnq04DK6FE36G2ihZTKbHfaJc/sz3LBIGnNa0Hwv+NZCYxNKsnm5IDBDx//c6btS/v1+6ESNE2YdieabitaPyH0RDBppIRcX2ufK6Fk71GydEz2pXkfnG8QN1zJRJn+PHf1Gg70SLF/aXHhGBVyudSlMk+EE89Or5ZyJLCSmUbS0LAHoBiVoSUtFb25iMSd/Zf3DsBPr/hZGKTfd7/c6BqeSKOidNPnOryVYThQM3hec5sbToDLneUqyhXRlAiifCw7x0he3XfFJp+NH0"; // Insert your own key here


    public void runOpMode() throws InterruptedException {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.colorSensor.get("color");
        front = hardwareMap.colorSensor.get("front");
        crservo = hardwareMap.crservo.get("servo");
        initHardware();

        setupVuforia();

        waitForStart();

        ods.enableLed(true);
        double initLightVal = ods.getLightDetected();
        double initColorVal = avg(front);

        moveInches(INIT_DRIVE_DISTANCE); //move to roughly the white line

        for(int i = 0; i < 2; i++) {
            left.setPower(LINE_FORWARD_POWER);
            right.setPower(LINE_FORWARD_POWER);
            //move so center is over white line
            while (ods.getLightDetected() - initLightVal < ODS_WHITE_THRESHOLD) {
//                int leftPos = left.getCurrentPosition();
//                int rightPos = right.getCurrentPosition();
//                if (leftPos > rightPos) {
//                    left.setPower(power * (1 - Math.min(100, leftPos - rightPos) / 100d));
//                    right.setPower(power);
//                } else if (rightPos > leftPos) {
//                    right.setPower(power * (1 - Math.min(100, rightPos - leftPos) / 100d));
//                    left.setPower(power);
//                }
                if (avg(front) > FRONT_WHITE_THRESHOLD) {
                    left.setPower(LINE_SLOW_POWER);
                    right.setPower(LINE_SLOW_POWER);
                    telemetry.addData("slowing down", true);
                    telemetry.update();
                }
            }


            left.setPower(0);
            right.setPower(0);
            Thread.sleep(200);

            //align robot along the white line
            if (RED_TEAM) {
                right.setPower(ALIGN_POWER);
                left.setPower(-ALIGN_POWER);
            } else {
                right.setPower(-ALIGN_POWER);
                left.setPower(ALIGN_POWER);
            }
            while (avg(front) < FRONT_WHITE_THRESHOLD) {
            }

            left.setPower(0);
            right.setPower(0);
            Thread.sleep(300);

            //align with images using vuforia
            visionTargets.activate();
            OpenGLMatrix latestLocation = getLocation();

            if (latestLocation != null) {
                lastKnownLocation = latestLocation;
            }

            int angle = getAngleFromMatrix(lastKnownLocation);
            int anglebuffer = 2; //tweak this

            while (Math.abs(angle) - 90 > anglebuffer) {
                if ((angle < 0 && angle > -90 + anglebuffer) || (angle > 0 && angle > 90 + anglebuffer)) {
                    left.setPower(ALIGN_POWER);
                    right.setPower(-ALIGN_POWER);
                } else if ((angle < 0 && angle < -90 - anglebuffer) || (angle > 0 && angle < 90 - anglebuffer)) {
                    left.setPower(-ALIGN_POWER);
                    right.setPower(ALIGN_POWER);
                }

                latestLocation = getLocation();
                if (latestLocation != null) {
                    lastKnownLocation = latestLocation;
                }
                angle = getAngleFromMatrix(lastKnownLocation);

                telemetry.addData("angle", angle);
                telemetry.update();
            }

            visionTargets.deactivate();

            left.setPower(0);
            right.setPower(0);


            //drive forward until colors
            cs.enableLed(true);
            sleep(500);
            left.setPower(FIND_BEACON_POWER);
            right.setPower(FIND_BEACON_POWER);
            while (avg(cs) < BEACON_FOUND_THRESHOLD) {
                sleep(30);
                telemetry.addData("Color", avg(cs));
                telemetry.update();
            }

            //stop before the wall
            sleep(100);
            left.setPower(0);
            right.setPower(0);

            //scan
            scanBeacon();

            //pressing the button
            right.setPower(PUSH_BUTTON_POWER);
            left.setPower(PUSH_BUTTON_POWER);
            sleep(1000);

            //back up!
            right.setPower(-PUSH_BUTTON_POWER);
            left.setPower(-PUSH_BUTTON_POWER);
            crservo.setPower(crPower);
            sleep(CR_CENTER_TIME);
            crservo.setPower(0);
            sleep(3000-CR_CENTER_TIME);
            right.setPower(0);
            left.setPower(0);

            if(i == 0){
                rotate(RED_TEAM?SECOND_TURN_ANGLE:-SECOND_TURN_ANGLE);
                moveInches(SECOND_BEACON_DISTANCE);
            }
        }
    }

    public double avg() {
        return avg(cs);
    }

    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }


    public void scanBeacon() {
        try {
            //set passive
            boolean wentLeft = true;
            cs.enableLed(false);
            sleep(500);
            double power = CR_POWER;
            crservo.setPower(power);

            //scan until correct or incorrect color
            while (Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))
                sleep(30);
            //turn if wrong color, else stay
            if (cs.red() > cs.blue() != RED_TEAM) {
                power = -power;
                crservo.setPower(power);
                sleep(400);
                wentLeft = false;
                // Small sleep so that it goes back into the center
                while (Math.abs(cs.red() - cs.blue()) <= 1)
                    sleep(30); // Go until you see one color more than the other
            }
            //error in blue edge
            if (!RED_TEAM)
                sleep(200);
            crservo.setPower(0);
            sleep(500);

            //button identification
            cs.enableLed(true);
            sleep(500);
            power /= 2;
            crservo.setPower(power);
            if (RED_TEAM) {
                while (avg() > CS_BLACK_THRESHOLD) { sleep(35); }
            } else {
                while (avg() > CS_BLACK_THRESHOLD) { sleep(35); }
            }
            if(wentLeft) {
                crPower = -crPower;
            }
            crservo.setPower(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setupVuforia() {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        // Setup the targets to be tracked
        //origin for coordinate system is set to the red corner.
        //+x direction is set to side without beacons,
        //+y direction set to side with beacons,
        //+z direction set to out of field.
        wheels = visionTargets.get(0); // 0 corresponds to the wheels target
        wheels.setName("Wheels");
        wheels.setLocation(createMatrix(2100, 3600, 150, 90, 0, 0));

        tools = visionTargets.get(1);
        tools.setName("Tools");
        tools.setLocation(createMatrix(0, 2700, 150, 90, 0, 90));

        legos = visionTargets.get(2);
        legos.setName("Legos");
        legos.setLocation(createMatrix(900, 3600, 150, 90, 0, 0));

        gears = visionTargets.get(3);
        gears.setName("Gears of War");//:P
        gears.setLocation(createMatrix(0, 1500, 150, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 0);

        // Setup listeners and informs them of phone information
        wheelListener = (VuforiaTrackableDefaultListener) wheels.getListener();
        wheelListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        toolListener = (VuforiaTrackableDefaultListener) tools.getListener();
        toolListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        legoListener = (VuforiaTrackableDefaultListener) legos.getListener();
        legoListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        gearListener = (VuforiaTrackableDefaultListener) gears.getListener();
        gearListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix getLocation(){
        OpenGLMatrix location = createMatrix(0, 0, 0, 0, 0, 0);//just set to orign since it'll get updated no matter what at this location on the field

        if(gearListener.isVisible()){ //if gears picture is visible set location based on that picture
            location = gearListener.getUpdatedRobotLocation();
        }else if(toolListener.isVisible()){ //if tools picture is visible set location based on that picture
            location = toolListener.getUpdatedRobotLocation();
        }else if(wheelListener.isVisible()){ //if wheels picture is visible set location based on that picture
            location = wheelListener.getUpdatedRobotLocation();
        }else if(legoListener.isVisible()) { //if legos picture is visible set location based on that picture
            location = legoListener.getUpdatedRobotLocation();
        }

        return location;

    }

    public int getAngleFromMatrix(OpenGLMatrix matrix){
        String m=formatMatrix(matrix);
        int start=m.indexOf('Z');
        int end=m.indexOf(' ', start+2);
        return parseInt(m.substring(start+2, end));
    }
    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }


}
