package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private HardwareMap hwmap = null;

    // MOTORS //
    private DcMotor fl = null, fr = null, bl = null, br = null; //Drive Motors

    private DcMotor lift; //Lift motor

    private DcMotor intake; //Sweeper motor

    public DcMotor hangLeft, hangRight; //Latch/Deploy motor

    //   SERVOS   //
    public Servo leftGate;
    public CRServo rightGate; //Gate control servos

    //VARIABLES //
    private double drivePower = 0, turnPower = 0;
    private double leftPower = 0, rightPower = 0;

    private double leftPos = 0, rightPos = 0;
    private final double tickToInch = 445.625/12, tickToDegree = 507/90;

    public double hangPos = 0;

    private double liftPower = 0;
    private double hangPower = 0;
    private double intakePower = 0;

    boolean leftGateOpen = false, rightGateOpen = false;

    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSED = 0;

    boolean hangUp = false;
    static final int HANG_UP = -14575;
    static final int HANG_DOWN = 0;

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

        resetHang();

        intake = hwmap.dcMotor.get("intake");

        lift = hwmap.dcMotor.get("lift");

        leftGate = hwmap.servo.get("rGate");
        rightGate = hwmap.crservo.get("lGate");

        hangLeft.setTargetPosition(HANG_DOWN);
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

    public void resetDrive() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } //Resets drive encoders

    public void resetHang() {
        hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } //Resets Drive Encoders

    public void updateDrive() {
        leftPos = (fl.getCurrentPosition() + bl.getCurrentPosition()) / 2;
        rightPos = (fr.getCurrentPosition() + br.getCurrentPosition()) / 2;
    } //Update Drive position

    public void updateHang() {
        hangPos = (hangLeft.getCurrentPosition() + hangRight.getCurrentPosition());
    } //Update Hang Position


    public void gateControl(boolean left, double rightPower) {
        leftGateOpen = left;
        if(leftGateOpen) {
            leftGate.setPosition(GATE_CLOSED);
        } else {
            leftGate.setPosition(GATE_OPEN);
        }

        rightGate.setPower(rightPower);
    } //Set Gate Status

    public void hangControl(double power) {
        hangPower = power;

        hangLeft.setPower(hangPower);
        hangRight.setPower(hangPower);
    } //Set Hang Power


    public void intakeControl(double power) {
        intakePower = power;
        intake.setPower(intakePower);
    } //Set Intake Power

    public void liftControl(double power) {
        liftPower = power;

        lift.setPower(liftPower);
    } //Set Lift Power
}
