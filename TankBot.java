/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

 /*
 TeleOp Mode
 Enables control of the robot via the gamepad
 */
 @TeleOp(name="TankBot", group ="Tank")


public class TankBot extends OpMode {
	DcMotor motorRightRear;
	DcMotor motorRightFront;
	DcMotor motorLeftRear;
	DcMotor motorLeftFront;

    // if robot is lopsided, weight adjustment value will compensate
    // RANGE [0, 1] as a percentage of the total power we want.
    // increase adjustment values for wheels that are on the heavier part of the robot
    // because we are scaling weightAdjust according to the power
    float weightAdjustRF = 1;
    float weightAdjustLR = 1;

    float weightAdjustRR = (float) 0.9;
    float weightAdjustLF = (float) 0.9;


	@Override
	public void init() {

        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");

        //set directions of motors when driving
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);
    }


	@Override
	public void loop() {

        // adds telemetry data --> shows values of motor positions
        telemetry.addData("motorLeftFront", motorLeftFront.getCurrentPosition());
        telemetry.addData("motorRightFront", motorRightFront.getCurrentPosition());

		/*
		 Gamepad 1 controls the motors via the left joystick
		 Gamepad 2 controls nothing right now
        */

        float yVal = gamepad1.left_stick_y; //left joystick controls all wheels
        float xVal = gamepad1.left_stick_x;
        float xValRight = gamepad1.right_stick_x; // x axis of the right joystick

        // negate all values since the y axis is reversed on the joypads and -1 should be 1
        yVal = -(yVal);

        // clip the right/left values so that the values never exceed +/- 1
        yVal = Range.clip(yVal, -1, 1);
        xVal = Range.clip(xVal, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        yVal = (float) scaleInput(yVal);
        xVal = (float) scaleInput(xVal);

        // set power to 0 if joysticks are at (0,0)
        if (yVal == 0 && xVal == 0) {
            motorRightFront.setPower(0.0);
            motorRightRear.setPower(0.0);
            motorLeftFront.setPower(0.0);
            motorLeftRear.setPower(0.0);
        }

        //Robot can travel forward, backward, right, and left
        // adjust x and y values according to the robot's weight distribution using weightAdjust


        // FORWARDS: all wheels must go backward
        // the joystick yVal will be positive when going forward
        // in this statement, yVal is negated because the wheels need to go backwards
        if (yVal > Math.abs(xVal)) {
            motorRightFront.setPower(-yVal * weightAdjustRF);
            motorRightRear.setPower(-yVal * weightAdjustRR);
            motorLeftFront.setPower(-yVal * weightAdjustLF);
            motorLeftRear.setPower(-yVal * weightAdjustLR);
        }

        // BACKWARDS: all wheels must go forward
        // the joystick yVal will be negative when going backward
        // in this statement, yVal is negated because the wheels need to go forwards
        if (yVal < Math.abs(xVal) && Math.abs(yVal) > Math.abs(xVal)) {
            motorRightFront.setPower(-yVal * weightAdjustRF);
            motorRightRear.setPower(-yVal * weightAdjustRR);
            motorLeftFront.setPower(-yVal * weightAdjustLF);
            motorLeftRear.setPower(-yVal * weightAdjustLR);
        }

        /* RIGHT: the right front and left rear wheels must go forward
           and the right rear and left front wheels must go backward */
        if (xVal > Math.abs(yVal)) {
            motorRightFront.setPower(xVal * weightAdjustRF); // 0 < xVal < 1
            motorRightRear.setPower(-xVal * weightAdjustRR);  // -1 < -xVal < 0
            motorLeftFront.setPower(-xVal * weightAdjustLF);
            motorLeftRear.setPower(xVal * weightAdjustLR);
        }

        /* LEFT: the right rear and left front wheels must go forward
           and the right front and left rear wheels must go backward */
        if (xVal < Math.abs(yVal) && Math.abs(xVal) > Math.abs(yVal)){
            motorRightFront.setPower(xVal * weightAdjustRF);  // -1 < xVal < 0
            motorRightRear.setPower(-xVal * weightAdjustRR); // 0 < -xVal < 1
            motorLeftFront.setPower(-xVal * weightAdjustLF);
            motorLeftRear.setPower(xVal * weightAdjustLR);
        }


        /* SPIN RIGHT: the left front and rear wheels go forward
           and the right front and rear wheels go backward

        if (xValRight > 0) {
            // left wheels forward
            motorLeftFront.setPower(-yVal * weightAdjustLF);
            motorLeftRear.setPower(-yVal * weightAdjustLR);
            // right wheels backward
            motorRightFront.setPower(-yVal * weightAdjustRF);
            motorRightRear.setPower(-yVal * weightAdjustRR);

        }
        */

        /* SPIN LEFT: the right front and rear wheels go forward
           and the left front and rear wheels go backward
         */
        /*
        if (xValRight < 0){
            // right wheels forward
            motorRightFront.setPower(-yVal * weightAdjustRF);
            motorRightRear.setPower(-yVal * weightAdjustRR);
            // left wheels backward
            motorLeftFront.setPower(-yVal * weightAdjustLF);
            motorLeftRear.setPower(-yVal * weightAdjustLR);
        }
        */


    }

	@Override
	public void stop() { // stops all motors when op mode is disabled from driver station
        motorRightFront.setPower(0.0);
        motorRightRear.setPower(0.0);
        motorLeftFront.setPower(0.0);
        motorLeftRear.setPower(0.0);
	}
	
	/*
	 This method scales the joystick input so for low joystick values, the
	 scaled value is less than linear.  This is to make it easier to drive
	 the robot more precisely at slower speeds.
	 */

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}
		
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}
		
		return dScale;
	}
//defaults to red, but basically this is just a template class
    String redOrBlue() {
        return "red";
    }

}

