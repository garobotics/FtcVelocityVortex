package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

enum State3 {INITIALIZE,
    LAUNCH_BALL,
    MOVE_TO_BEACON,
    TURN_TO_BEACON,
    LINE_FOLLOW,
    SQUARE_UP,
    DETECT_COLOR,
    PUSH_BUTTON,
    DRIVE_TO_PLATFORM,
    PARK_ON_PLATFORM,
    STOP
}
// possible values for state because it is an enum type

@Autonomous(name="State Machine 3")
@Disabled


public class stateMachine3 extends HardwareClass {

    State3 state;    // declares variable state of type State
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state



    @Override
    public void init() {

        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");
        ballFlipper = hardwareMap.dcMotor.get("flip");

        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        // assigns state variable to enum INITIALIZE
        state = State3.INITIALIZE;



        // reset encoder target positions to 0 on drive wheels and ball flipper
        resetDriveEncoders();
        ballFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set drive power to 0
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);

        telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                motorRightFront.getCurrentPosition(),
                motorLeftFront.getCurrentPosition()));



    }

    private void changeState(State3 newState)
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

        switch(state) {
            case INITIALIZE:

                // wait 1 hardware cycle
                // motorRightFront.waitOneFullHardwareCycle();
                // motorLeftFront.waitOneFullHardwareCycle();
                // motorRightRear.waitOneFullHardwareCycle();
                // motorLeftRear.waitOneFullHardwareCycle();

                // if both encoders are close to 0, start moving and change state to next state
                if ((Math.abs(motorRightFront.getCurrentPosition()) < 5) && (Math.abs(motorLeftFront.getCurrentPosition()) < 5)) {

                    ballFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ballFlipper.setTargetPosition(1120);
                    ballFlipper.setPower(1.0);
                    changeState(State3.LAUNCH_BALL);
                }
                else {
                    // continue printing telemetry data to the phone (in main loop)
                    telemetry.addData("0", String.format("State: INITIALIZE"));

                }


            case LAUNCH_BALL:
                // if ball flipper is done, start moving and change state to next state
                if (ballFlipper.getCurrentPosition() >= 1120) {
                    ballFlipper.setPower(0);
                    motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightFront.setTargetPosition(5000);
                    motorLeftFront.setTargetPosition(5000);
                    motorLeftFront.setPower(1);
                    motorLeftRear.setPower(1);
                    motorRightFront.setPower(1);
                    motorRightRear.setPower(1);
                    telemetry.addData("0", String.format("The eagle is in the nest"));
                    changeState(State3.MOVE_TO_BEACON);
                }
                else {
                    telemetry.addData("0", String.format("State: LAUNCH_BALL"));
                }

            case MOVE_TO_BEACON:
                telemetry.addData("0", "The eagle has flown.");
                if ((motorRightFront.getCurrentPosition() > 5000) || (motorLeftFront.getCurrentPosition() > 5000)) {
                    motorRightFront.setPower(0.0);
                    motorLeftFront.setPower(0.0);
                    motorRightRear.setPower(0.0);
                    motorLeftRear.setPower(0.0);
                    changeState(State3.STOP);
                }
                else {
                    telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));
                }


            /*case MOVE_TO_BEACON:
                if ((motorRightFront.getCurrentPosition() < -10000) || (motorLeftFront.getCurrentPosition() < -10000)) {
                    motorRightFront.setPower(0.0);
                    motorLeftFront.setPower(0.0);
                    motorRightRear.setPower(0.0);
                    motorLeftRear.setPower(0.0);
                    changeState(State3.STOP);

                }
                else {
                    // continue printing telemetry data to the phone (in main loop)
                    telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));
//                     motorRightFront.setPower(-1);
                    //                    motorLeftFront.setPower(-1);
                    //                    motorRightRear.setPower(-1);
                    //                    motorLeftRear.setPower(-1);
                } */



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