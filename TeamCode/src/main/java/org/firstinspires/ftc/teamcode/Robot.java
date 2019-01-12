package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private HardwareMap hwmap = null;

    // MOTORS // Hello Darkness My old friend
    private DcMotor fl = null, fr = null, bl = null, br = null; //Drive Motors

    private DcMotor lift; //Lift motor

    private DcMotor intake; //Sweeper motor

    private DcMotor hangLeft, hangRight; //Latch/Deploy motor

    //VARIABLES //
    private double drivePower = 0, turnPower = 0;
    private double leftPower = 0, rightPower = 0;

    private double leftPos = 0, rightPos = 0;
    private final double tickToInch = 445.625/12, tickToDegree = 507/90;

    public void init(HardwareMap ahwmap) {
        hwmap = ahwmap;

        fl = hwmap.get(DcMotor.class, "frontLeft");
        bl = hwmap.get(DcMotor.class, "backLeft");
        fr = hwmap.get(DcMotor.class, "frontRight");
        br = hwmap.get(DcMotor.class, "backRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        resetDrivePosition();

        hangRight = hwmap.dcMotor.get("hang1");
        hangRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hangLeft = hwmap.dcMotor.get("hang2");

        hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hwmap.dcMotor.get("intake");

        lift = hwmap.dcMotor.get("lift");
    } //Initializes All Motors and Servos

    public double driveDistance() {
        double rightDist = rightPos / tickToInch;
        double leftDist = leftPos / tickToInch;

        return (rightDist + leftDist)/2;
    } // Outputs distance robot has traveled

    public double turnAngle() {
        double rightAngle = rightPos / tickToDegree;
        double leftAngel = leftPos / tickToDegree;

        return (rightAngle - leftAngel)/2;
    } //Outputs angle robot has traveled

    public void drive(double dP, double tP) {
        drivePower = dP; turnPower = tP;

        rightPower = drivePower + turnPower;
        leftPower = drivePower - turnPower;

        fl.setPower(leftPower);
        bl.setPower(leftPower);
        fr.setPower(rightPower);
        br.setPower(rightPower);
    } //Controls drive motors

    public void resetDrivePosition() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } //Resets drive encoders

    public void updateDrivePosition() {
        leftPos = (fl.getCurrentPosition() + bl.getCurrentPosition())/2;
        rightPos = (fr.getCurrentPosition() + br.getCurrentPosition())/2;
    } //Updates drive variables

    public void updateDriveTelemetry(Telemetry t) {
        t.addData("Drive/Turn Power", drivePower + "/" + turnPower);
        t.addData("L/R Position", leftPos + "/" + rightPos);
        t.addData("Distance", driveDistance());
        t.addData("Angle", turnAngle());
        t.addData("L/R Power", leftPower + "/" + rightPower);
    } //Updates Drive Telemetry

    public void updateRobotTelemetry(Telemetry t) {
        updateDriveTelemetry(t);
        t.addData("Hang Pos", hangLeft.getCurrentPosition());
    }
}
