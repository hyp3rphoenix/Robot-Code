package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;

@Autonomous (name = "Marker Auto")
public class Marker_Auto extends LinearOpMode {

    Robot r = new Robot();
    Sensors s = new Sensors();

    // VUFORIA VARIABLES //
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZdS+Pv/////AAABmWlorWI7REqsgTLWFnbiFDiNVsM52BGqaCJz4loFGfpOD8yOuSq+qcS7ujWqui9+z7H0Xe6TA8D0WLzDB7s725J/K+g43UjlW8fU3qhsDXKR/oRWAA7q+gGNUL/yKHh6vc9C0EHwaoF/2uvAsCWX6rQ2xGLSZgFco23PDKiuh/vWYI90kX0oihz94nyp/4oYWC2tmdpKhHQ78KkXN7MJImgKyvoCjSyWdDARRKadmhVsTRE9YxpDCGtiA5P456MOerXfWhPz+OpC+EGvmVUIrFR//V+z1qd5G0AcDyq8VodTVf3b6AnexrpM5NWeI4ovAC5Zo7Jgq9Qr6pBJTOuLCD/lrqm/3rqba1qlHqBvpYz3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    // AUTO VARIABLES //
    String color = "N/A";
    int goldPos = -1; // 0 Left - 1 Mid - 2 Right

    enum MarkerAuto {
        DROP, DRIVE_FROM_LANDER, DRIVE_FROM_LANDER_2, TURN_TO_CAMERA, SCAN_MID, DRIVE_TO_RIGHT, SCAN_RIGHT, DRIVE_TO_LEFT, SCAN_LEFT, TURN_TO_SAMPLE, FINISHED, LIFT_DOWN, NULL
    }

    HashMap<String, PIDController> PIDs = new HashMap<String, PIDController>(); // Key = Name, V = Controller

    PIDController driveFWD = new PIDController();
    PIDController turnSample = new PIDController();
    PIDController cratorPID = new PIDController();

    StallDetect stallHang = new StallDetect();

    @Override
    public void runOpMode() {

        MarkerAuto state = MarkerAuto.DROP;
        MarkerAuto secondary = MarkerAuto.NULL;
        r.init(hardwareMap);
        s.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        stallHang.init(0);
        stallHang.setTolerance(5.00);

        driveFWD.setSensitivity(25);
        cratorPID.setSensitivity(25);
        turnSample.setSensitivity(25);

        while (opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() >= 1) {
                        if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                            color = "gold";
                        } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
                            color = "silver";
                        } else {
                            color = "none";
                        }
                    }
                }
            }

            s.updateSensors();
            r.updateDrivePosition();
            r.updateHang();

            if(state.equals(MarkerAuto.DROP)) {
                r.hangControl(1.0);

                stallHang.runStallDetect(r.hangPos);

                if(stallHang.stalled) {
                    state = MarkerAuto.DRIVE_FROM_LANDER;

                    driveFWD.setupPID(0.45, 0.00005, -0.0020);
                    driveFWD.initPID(120, 0, 20);
                    r.hangControl(0);
                }
            }
//
            if(state.equals(MarkerAuto.SCAN_MID)) {
                sleep(500);

                if(color.equalsIgnoreCase("gold")) {
                    goldPos = 1;
                    driveFWD.setupPID(-0.25, 0, -0.000005);
                    driveFWD.initPID(-3, 0, 0.5);
                    state = MarkerAuto.DRIVE_FROM_LANDER;
                }

                else {
                    state = MarkerAuto.DRIVE_FROM_LANDER;
                }
            }

            if(state.equals(MarkerAuto.DRIVE_FROM_LANDER)) {
                driveFWD.runPID(r.turnAngle());
                r.drive(0, driveFWD.output);
                if(driveFWD.complete) {
                    r.drive(0, 0);
                    sleep(500);
                    cratorPID.setupPID(-0.4, 0, 0.000005);
                    cratorPID.initPID(-36, 0, 5);
                    r.resetDrivePosition();
                    state = MarkerAuto.DRIVE_FROM_LANDER_2;
                }
            }

            if(state.equals(MarkerAuto.DRIVE_FROM_LANDER_2)) {
                cratorPID.runPID(r.driveDistance());
                r.drive(cratorPID.output, 0);

                if(cratorPID.complete) {
                    r.drive(0, 0);

                    state = MarkerAuto.FINISHED;
                }
            }

            if (state.equals(MarkerAuto.SCAN_LEFT)) {
                sleep(500);

                if(goldPos == -1) {
                    updateTfod();

                    if (color.equalsIgnoreCase("gold")) {
                        state = MarkerAuto.DRIVE_FROM_LANDER;
                    } else if(color.equalsIgnoreCase("silver")) {
                        state = MarkerAuto.DRIVE_TO_RIGHT;
                    }
                } else {
                    state = MarkerAuto.TURN_TO_SAMPLE;
                }
            }



            if(state.equals(MarkerAuto.FINISHED)) {
                r.drive(0, 0);
                r.hangControl(0);
            }

            if(secondary.equals(MarkerAuto.LIFT_DOWN) && !state.equals(MarkerAuto.DROP)) {
                r.hangControl(-1.0);

                if(r.hangPos >= 5) {
                    r.hangControl(0);
                    secondary = MarkerAuto.NULL;
                }
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("State", state.toString());
            //s.composeIMUTelemetry(telemetry);
            r.updateRobotTelemetry(telemetry);
            //stallHang.stallTelemetry(telemetry);
            driveFWD.PIDTelemetry(telemetry);
            telemetry.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void updateTfod() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() >= 1) {
                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                        color = "gold";
                    } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
                        color = "silver";
                    } else {
                        color = "none";
                    }
                }
            }
        }
    }

    private void addPIDs(String name, double kp, double ki, double kd) {
        PIDs.put(name, new PIDController(kp, ki, kd));
    }
}


