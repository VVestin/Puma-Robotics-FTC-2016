package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

/*
 * This OpMode was written for the Vuforia Basics video. This demonstrates basic principles of
 * using Vuforia in FTC.
 */
@Autonomous(name = "Vuforia")
public class TestVuforiaOp extends LinearOpMode
{
    // Variables to be used for later
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

    public static final String VUFORIA_KEY = "AV4ONxv/////AAAAGefaDmLKjkgWifNHOt4h8QgFb23EhhiUz7Po/rcnXDMuJHa2Okvh/NLUSza5phLaIuyvWUINyu/cyKpmChyUCJ/A05QHnq04DK6FE36G2ihZTKbHfaJc/sz3LBIGnNa0Hwv+NZCYxNKsnm5IDBDx//c6btS/v1+6ESNE2YdieabitaPyH0RDBppIRcX2ufK6Fk71GydEz2pXkfnG8QN1zJRJn+PHf1Gg70SLF/aXHhGBVyudSlMk+EE89Or5ZyJLCSmUbS0LAHoBiVoSUtFb25iMSd/Zf3DsBPr/hZGKTfd7/c6BqeSKOidNPnOryVYThQM3hec5sbToDLneUqyhXRlAiifCw7x0he3XfFJp+NH0"; // Insert your own key here

    public void runOpMode() throws InterruptedException{
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        int anglebuffer = 3; //tweak this

        while (opModeIsActive()) {
            OpenGLMatrix latestLocation=getLocation();
            if (latestLocation != null) {
                lastKnownLocation = latestLocation;
            }
            int angle = getAngleFromMatrix(lastKnownLocation);
            telemetry.addData("Tracking: " + wheels.getName(), wheelListener.isVisible());
            telemetry.addData("Tracking: " + gears.getName(), gearListener.isVisible());
            telemetry.addData("Tracking: " + legos.getName(), legoListener.isVisible());
            telemetry.addData("Tracking: " + tools.getName(), toolListener.isVisible());
            telemetry.addData("angle", angle);
            telemetry.addData("loc ", formatMatrix(lastKnownLocation));
            telemetry.update();
        }
//        while(opModeIsActive()){
//            // Ask the listener for the latest information on where the robot is
//            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
//
//            // The listener will sometimes return null, so we check for that to prevent errors
//            if(latestLocation != null)
//                lastKnownLocation = latestLocation;
//
//            // Send information about whether the target is visible, and where the robot is
//            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
//            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
//
//            // Send telemetry and idle to let hardware catch up
//            telemetry.update();
//            idle();
//        }
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
        // Origin for coordinate system is set to the red corner.
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
        phoneLocation = createMatrix(0, 0, 200, 90, 0, 0);

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
        if(gearListener.isVisible()){ // If gears picture is visible set location based on that picture
            location = gearListener.getUpdatedRobotLocation();
        }else if(toolListener.isVisible()){ // If tools picture is visible set location based on that picture
            location = toolListener.getUpdatedRobotLocation();
        } else if(wheelListener.isVisible()){ // If wheels picture is visible set location based on that picture
            location = wheelListener.getUpdatedRobotLocation();
        }else if(legoListener.isVisible()) { // If Legos picture is visible set location based on that picture
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
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }
}