package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ga on 2/3/17.
 */

public class newLinearOpMode extends LinearOpMode {

    public void driveToBeacon() {
        motorRightFront.setTargetPosition(-10000);
        motorLeftFront.setTargetPosition(-10000);
        motorRightRear.setTargetPosition(-10000);
        motorLeftRear.setTargetPosition(-10000);
        motorRightFront.setPower(-1);
        motorLeftFront.setPower(-1);
        motorRightRear.setPower(-1);
        motorLeftRear.setPower(-1);

        if ((motorRightFront.getCurrentPosition() < -10000) || (motorLeftFront.getCurrentPosition() < -10000)) {
            // return to the main method
        }

    }

    public void turnToBeacon() {

    }

    public void turnRight() {

    }

    public void turnLeft() {

    }

}
