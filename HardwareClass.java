package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ga on 11/7/16.
 */

public abstract class HardwareClass extends OpMode {
    public DcMotor motorRightRear;
    public DcMotor motorRightFront;
    public DcMotor motorLeftRear;
    public DcMotor motorLeftFront;
    public DcMotor ballFlipper;
    public DcMotor sweeper;

    public float weightAdjustRF = 1;
    public float weightAdjustLR = 1;

    public float weightAdjustRR = (float) 0.9;
    public float weightAdjustLF = (float) 0.9;
}
