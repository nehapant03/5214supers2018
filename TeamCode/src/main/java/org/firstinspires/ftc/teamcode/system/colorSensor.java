package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.ColorSensor;

class colorSensor {
    final double currentRatio = 1.3;
    private ColorSensor colorSensor;

    public colorSensor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public String checkColor() {
        double redOverBlue = (colorSensor.red()+1) / (colorSensor.blue() + 1);
        if (redOverBlue >= currentRatio) {
            //if it is greater than ratio, it is red
            return "RED";
        }
        else if (redOverBlue <= currentRatio) {
            //if it is less than ratio, it is blue
            return "BLUE";
        }
        else {
            //if nothing is detected, return not defined
            return "UNDEF";
        }
    }
}
