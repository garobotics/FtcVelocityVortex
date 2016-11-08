package org.firstinspires.ftc.robotcontroller.external;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareClass;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ga on 3/7/16.
 */

 enum State {INITIALIZE,
            MOVE_TO_BEACON,
            TURN_TO_BEACON,
            GET_TO_BEACON,
            //LINE_FOLLOW,
            //SQUARE_UP,
            DETECT_COLOR,
            PUSH_BUTTON,
            DRIVE_TO_PLATFORM,
            PARK_ON_PLATFORM,
            STOP
            }
// possible values for state because it is an enum type

@Autonomous(name="State Machine", group ="State Machines")


public class StateMachine extends HardwareClass {

    // declare common global variables

     State state;    // declares variable state of type State

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    public double robotX;
    public double robotY;
    public double robotBearing;

    public List<VuforiaTrackable> allTrackables;



    @Override
     public void init() {

         motorRightFront = hardwareMap.dcMotor.get("rf");
         motorRightRear = hardwareMap.dcMotor.get("rb");

         motorLeftFront = hardwareMap.dcMotor.get("lf");
         motorLeftRear = hardwareMap.dcMotor.get("lb");

         motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
         motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
         motorRightFront.setDirection(DcMotor.Direction.REVERSE);
         motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        // assigns state variable to enum INITIALIZE
        state = State.INITIALIZE;

        // reset encoder target position to 0
  //      motorRightFront.setTargetPosition(0);
  //      motorLeftFront.setTargetPosition(0);
        resetDriveEncoders();

        // set drive power to 0
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);

        telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                motorRightFront.getCurrentPosition(),
                motorLeftFront.getCurrentPosition()));


        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;


        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AY7wJZH/////AAAAGZMdB1/VKEVukhg26LQV42lceWeAr1LKIKyASsN63SUKG2y0cw4j0jxeOOY3MqgM0teJz8kyQGCaPpFEu0kXsblybBfCo+Ta0PZapJYWFCzk+NdiJIK7iy29OqFh7/vFMrdcl6i1iVX4We5Xvjr2XpYoJFd2m2RrUFrU6+vmv3RYYmLJynLI3IGP1jpHU6XZVPukzimvB1ABs6AelwYwUHzlXX/tloA4PuTLhhwUYRIzX948sQUr6Vr26fnZWPHLY/rJ0HyyTPaIUVro+giCdp8rVQoYBKbu+f7UTuN7r1H/XvyofXR6OlFLHi0SdQy91sRr3ER8I6iY19OwkhBOqQMzcpu6DK7A7Lik0J/EOnS1\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */

        VuforiaTrackables ftcPics = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTarget1 = ftcPics.get(1);
        redTarget1.setName("RedTarget1");  // tools

        VuforiaTrackable blueTarget1 = ftcPics.get(2);
        blueTarget1.setName("BlueTarget1");  // legos

        VuforiaTrackable redTarget2 = ftcPics.get(3);
        redTarget2.setName("RedTarget2");  // gears

        VuforiaTrackable blueTarget2 = ftcPics.get(0);
        blueTarget2.setName("BlueTarget2");  // wheels

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcPics);




        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        //  float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */

        /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */


        // red target 1
        OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation((float) -1803.4, (float) 901.7, 150)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget1.setLocation(redTarget1LocationOnField);
        RobotLog.ii(TAG, "Red Target1=%s", redTarget1LocationOnField.formatAsTransform());


        // red target 2
        OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation((float) -1803.4, (float) -300.567, 150)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget2.setLocation(redTarget2LocationOnField);
        RobotLog.ii(TAG, "Red Target2=%s", redTarget2LocationOnField.formatAsTransform());

        // blue target 1
        OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation((float) -901.7, (float) 1803.4, 150)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget1.setLocation(blueTarget1LocationOnField);
        RobotLog.ii(TAG, "Blue Target1=%s", blueTarget1LocationOnField.formatAsTransform());

        // blue target 2
        OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation((float) 300.567, (float) 1803.4, 150)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget2.setLocation(blueTarget2LocationOnField);
        RobotLog.ii(TAG, "Blue Target2=%s", blueTarget2LocationOnField.formatAsTransform() );


        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", phoneLocationOnRobot.formatAsTransform());

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /** Start tracking the data sets we care about. */
        ftcPics.activate();

     }

    private void changeState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        state = newState;
    }

    @Override
    public void loop() {
         telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                 motorRightFront.getCurrentPosition(),
                 motorLeftFront.getCurrentPosition()));

        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //


            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                robotX = trans.get(0);
                robotY = trans.get(1);
                robotBearing = rot.thirdAngle;

            }
        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", lastLocation.formatAsTransform());
        } else {
            telemetry.addData("Pos", "Unknown");
        }
        telemetry.update();



         switch(state) {
             case INITIALIZE:

                 // wait 1 hardware cycle
                 // motorRightFront.waitOneFullHardwareCycle();
                 // motorLeftFront.waitOneFullHardwareCycle();
                 // motorRightRear.waitOneFullHardwareCycle();
                 // motorLeftRear.waitOneFullHardwareCycle();

                 // if both encoders are close to 0, start moving and change state to MOVE_TO_BEACON
                 if ((Math.abs(motorRightFront.getCurrentPosition()) < 5) && (Math.abs(motorLeftFront.getCurrentPosition()) < 5)) {
                     setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                     motorRightFront.setTargetPosition(-10000);
                     motorLeftFront.setTargetPosition(-10000);
                     motorRightRear.setTargetPosition(-10000);
                     motorLeftRear.setTargetPosition(-10000);
                     motorRightFront.setPower(-1);
                     motorLeftFront.setPower(-1);
                     motorRightRear.setPower(-1);
                     motorLeftRear.setPower(-1);
                     changeState(State.MOVE_TO_BEACON);
                 }
                 else {
                     // continue printing telemetry data to the phone (in main loop)
                     telemetry.addData("0", String.format("State: INITIALIZE"));

                 }

                 break;


             case MOVE_TO_BEACON:
                 if ((motorRightFront.getCurrentPosition() < -10000) || (motorLeftFront.getCurrentPosition() < -10000)) {

                         // spin left
                         motorLeftFront.setPower(-1 * weightAdjustLF);
                         motorLeftRear.setPower(-1 * weightAdjustLR);

                         motorRightFront.setPower(1 * weightAdjustRF);
                         motorRightRear.setPower(1 * weightAdjustRR);

                         changeState(State.TURN_TO_BEACON);
                 }
                 else {
                     // continue printing telemetry data to the phone (in main loop)
                     telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));

                 }


             case TURN_TO_BEACON:
                 if (lastLocation != null) { // if the robot can see a target
                     motorRightFront.setPower(0.0);
                     motorLeftFront.setPower(0.0);
                     motorRightRear.setPower(0.0);
                     motorLeftRear.setPower(0.0);
                     changeState(State.STOP);

                 }
                 else {
                     telemetry.addData("0", String.format("State: TURN_TO_BEACON"));
                 }

             case GET_TO_BEACON:
                 if (true) {

                 }
                 else{
                 }




             case STOP:
                 telemetry.addData("0", String.format("State: STOP"));
                 break;
         }
    }

    // set up path segments to get to beacons
    // draw state transition diagram


    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
     //need this?   setEncoderTarget(0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotor.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (motorLeftFront.getMode() != mode)
            motorLeftFront.setMode(mode);

        if (motorRightFront.getMode() != mode)
            motorRightFront.setMode(mode);

        if (motorLeftRear.getMode() != mode)
            motorLeftRear.setMode(mode);

        if (motorRightRear.getMode() != mode)
            motorRightRear.setMode(mode);
    }

}
