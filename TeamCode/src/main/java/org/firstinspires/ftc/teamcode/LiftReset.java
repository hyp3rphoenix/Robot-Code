package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Reset")
public class LiftReset extends LinearOpMode {

    Robot r = new Robot();



    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        r.hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                r.hangControl(1.0);
            } else if(gamepad1.dpad_down) {
                r.hangControl(-1.0);
            } else {
                r.hangControl(0);
            }
        }
    }
}
