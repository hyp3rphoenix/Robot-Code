package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "2 Driver Teleop")
public class Teleop extends LinearOpMode {

    Robot r = new Robot();
    Sensors sensor = new Sensors();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        sensor.init(hardwareMap);

        r.init(hardwareMap);
        double drivePower = 0;
        double turnPower = 0;
        double deadzone = 0.2;

        double liftPower = 0;
        double intakePower = 0;
        double hangPower = 0;

        boolean leftGate = false, rightGate = false;

        waitForStart();

        while(opModeIsActive()) {

            // DRIVE CONTROLS
            drivePower = gamepad1.left_stick_y + (-gamepad2.left_stick_y/1.5);
            turnPower = gamepad1.right_stick_x + (gamepad2.right_stick_x);

            if(Math.abs(drivePower) < deadzone) {
                drivePower = 0;
            }

            if(Math.abs(turnPower) < deadzone) {
                turnPower = 0;
            }

            if(Math.abs(turnPower) > 0) {
                turnPower = turnPower;
            }

            if(gamepad1.dpad_up) {
                r.resetDrive();
            }

            //LIFT CONTROLS
            if(gamepad2.right_trigger > deadzone) {             //G2 Right Trigger Up
                liftPower = gamepad2.right_trigger;
            } else if(gamepad2.left_trigger > deadzone) {       //G2 Left Trigger Down
                liftPower = -gamepad2.left_trigger;
            } else {
                liftPower = 0;
            }

            //INTAKE CONTROLS
            if(gamepad2.right_bumper) {           //Right Bumper In
                intakePower = 1.0;
            } else if(gamepad2.left_bumper) {     //Left Bumper Out
                intakePower = -1.0;
            } else if(gamepad1.right_trigger > deadzone) {           //Right Bumper In
                intakePower = gamepad1.right_trigger;
            } else if(gamepad1.left_trigger > deadzone) { //Left Bumper Out
                intakePower = -gamepad1.left_trigger;
            } else {
                intakePower = 0;
            }

            if(gamepad2.right_bumper) {           //Right Bumper In
                intakePower = 1.0;
            } else if(gamepad2.left_bumper) {     //Left Bumper Out
                intakePower = -1.0;
            }

            //GATE CONTROLS
            leftGate = gamepad2.x;                              //G2 X Open Left
            rightGate = gamepad2.b;                             //G2 B Open Right

            //HANG CONTROLS
            if(gamepad1.dpad_up) {             //G1 Right Trigger Hang
                r.setHangTarget(r.HANG_UP);
            } else if(gamepad1.dpad_down) {       //G1 Left Trigger Drop
               r.setHangTarget(r.HANG_DOWN);
            }

            if(gamepad1.back) {
                r.resetDrive();
            }


            //CONTROL FUNCTIONS
            r.drive(drivePower, turnPower);
            r.hangControl(1.0);
            r.liftControl(liftPower);
            r.intakeControl(intakePower);
            r.gateControl(leftGate, rightGate);
            r.hangControl(hangPower);

            r.updateDrive();

            telemetry.addData("Status", "Running");
            r.updateRobotTelemetry(telemetry);
            sensor.composeIMUTelemetry(telemetry);
            telemetry.update();
        }
    }
}
