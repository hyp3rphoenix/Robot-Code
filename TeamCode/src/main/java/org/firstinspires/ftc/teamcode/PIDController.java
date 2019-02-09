package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    private double kP = 0, kI = 0, kD = 0;

    private double value = 0;
    private double minValue = 0;
    private double target = 0;

    private double error = 0;

    private double prevError = 0;
    private double totalError = 0;

    private double threhhold = 0;

    public double output = 0;

    private double counter = 0;
    private double sensitivity = 100;

    public boolean complete = false;

    public PIDController(double kp, double ki, double kd) {
        setupPID(kp, ki, kd);
    }

    public PIDController() {
        setupPID(0, 0, 0);
    }

    public void setupPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    public void initPID(double target, double startValue, double thresh) {
        this.target = target;
        minValue = startValue;

        prevError = target - minValue;

        totalError = 0;

        threhhold = thresh;

        complete = false;
    }

    public void setSensitivity (double val) {
        sensitivity = val;
    }

    public void runPID(double val) {
        value = val;

        error = (target - minValue) - value;

        double dError = error - prevError;

        totalError += error;


        double PComponent = (error/(target - minValue)) * kP;
        double IComponent = totalError * kI;
        double DComponent = dError * kD;

        output = PComponent + IComponent - DComponent;

        if(Math.abs(error) < threhhold) {
            counter ++;
        } else {
            counter = 0;
        }

        if(counter >= sensitivity) {
            complete = true;
        }
    }

    public void PIDTelemetry(Telemetry t) {
        t.addData("Settings", "kp: " + kP + "  ki" + kI + "  kd" + kD);
        t.addData("Target", target);
        t.addData("Running Vars", "e:" + error + "  ei:" + totalError + "  de" + (error - prevError));
        t.addData("Complete", complete + "  " + counter + "/" + sensitivity);
    }
}
