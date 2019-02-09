package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;

@Autonomous (name = "PID Tester")
public class PIDTester extends LinearOpMode{

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

    enum MarkerAuto {
        START, RUN, NULL
    }

    HashMap<String, PIDController> PIDs = new HashMap<String, PIDController>(); // Key = Name, V = Controller

    PIDController testPID = new PIDController();

    @Override
    public void runOpMode() {

        MarkerAuto state = MarkerAuto.START;
        r.init(hardwareMap);
        s.init(hardwareMap);

        testPID.setupPID(0.25, 0, 0);
        testPID.setSensitivity(25);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateTfod();
            s.updateSensors();
            r.updateDrivePosition();
            r.updateHang();

            if(state.equals(MarkerAuto.START)) {
                testPID.initPID(36, 0, 3);
                state = MarkerAuto.RUN;
            }

            if(state.equals(MarkerAuto.RUN)) {
                testPID.runPID(r.driveDistance());

                r.drive(testPID.output, 0);

                if(testPID.complete) {
                    state = MarkerAuto.NULL;
                }
            }

            if(state.equals(MarkerAuto.NULL)) {
                r.drive(0, 0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("State", state.toString());
            //s.composeIMUTelemetry(telemetry);
            r.updateDriveTelemetry(telemetry);
            testPID.PIDTelemetry(telemetry);
            telemetry.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

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
}