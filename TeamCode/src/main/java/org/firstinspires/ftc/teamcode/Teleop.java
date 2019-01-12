package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Teleop")

public class Teleop extends LinearOpMode {
    Robot robot = new Robot();

    public double drivePower = 0, turnPower = 0;

    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Running");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            robot.updateDrivePosition();

            drivePower = gamepad1.left_stick_y;
            turnPower = gamepad1.right_stick_x;

            robot.drive(drivePower, turnPower);

            telemetry.addData("Status", "Running");
            robot.updateDriveTelemetry(telemetry);
            telemetry.update();
        }
    }
}
