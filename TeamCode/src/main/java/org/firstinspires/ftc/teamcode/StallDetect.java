package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StallDetect {

    private double tolerance = 0.05;

    private int sensitivity = 15;

    private int counter = 0;

    private double prevVal = 0;

    double value = 0;

    public boolean stalled = false;

    public StallDetect() {}

    public void init(double val) {
        prevVal = val;
    }

    public void runStallDetect(double val) {
        value = val;
        double dVal = value - prevVal;

        prevVal = value;

        if(Math.abs(dVal) <= tolerance) {
            counter++;
        } else {

        }

        if(counter >= sensitivity) {
            stalled = true;
        }
    }

    public void setTolerance(double val) {
        tolerance = val;
    }

    public void setSensitivity(int val) {
        sensitivity = val;
    }

    public void stallTelemetry(Telemetry t) {
        t.addData("DVal", value + " - " + prevVal + " = " + (value - prevVal));
        t.addData("Tolerance", tolerance);
        t.addData("Completion", counter + "/" + sensitivity);
    }
}
