package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class motorWorld extends LinearOpMode{
    DcMotor motor;
    private int ticks;
    private int position2move2;
    public motorWorld(DcMotor motor) {
        this.motor = motor;
    }
    public void motorWithEncoder(double power, int inches) {
        ticks = (int) (inches * 1120 / (4 * 3.14159)); //converts inches to ticks
//        telemetry.addData("ticks: ", ticks);
        telemetry.update();

        //modifies moveto position based on starting ticks position, keeps running tally
        position2move2 = motor.getCurrentPosition() + ticks;
        motor.setTargetPosition(position2move2);
        motor.setPower(power);

    }
}
