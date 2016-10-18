package org.firstinspires.ftc.robotcontroller.external;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

 /**
 * Created by ga on 3/7/16.
 */

 enum State {INITIALIZE,
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

@Autonomous(name="State Machine", group ="State Machines")


public class StateMachine extends OpMode {

     DcMotor motorRightRear;
     DcMotor motorRightFront;
     DcMotor motorLeftRear;
     DcMotor motorLeftFront;
     State state;    // declares variable state of type State

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state



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

     }

    private void changeState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        state = newState;
    }

  //  public void start() {}

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
                     motorRightFront.setPower(0.0);
                     motorLeftFront.setPower(0.0);
                     motorRightRear.setPower(0.0);
                     motorLeftRear.setPower(0.0);
                     changeState(State.STOP);

                 }
                 else {
                     // continue printing telemetry data to the phone (in main loop)
                     telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));
//                     motorRightFront.setPower(-1);
 //                    motorLeftFront.setPower(-1);
 //                    motorRightRear.setPower(-1);
 //                    motorLeftRear.setPower(-1);
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
