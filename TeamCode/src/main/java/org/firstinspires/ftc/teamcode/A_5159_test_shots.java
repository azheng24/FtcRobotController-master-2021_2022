/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Autonomous test shots", group = "SDR")
//@Disabled
public class A_5159_test_shots extends LinearOpMode {


    /* Declare OpMode members. */
    Chassis_KLNavX_6109_v16a chassis = new Chassis_KLNavX_6109_v16a();   // Use Omni-Directional drive system
    Launcher_5159_v1 launcher = new Launcher_5159_v1();   // Use all other motors

    int factor;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    int stack_size = 0;

    //vuforia

    private static final String VUFORIA_KEY =
            "AVg5izT/////AAABmWMPqi8r1E2cnOx/70T+i+59OSJC7BfN2cb+ETeATfjrLzau9jsztE8DWwVcjr+nZ+z8VbYjWNrYU3/3I+dumAcLSILIXH7zHCk207cJOIbbLmNc26+gSg+yEOYerK/nhBUNAauKjIg27Q9eyfZkWPbNVqHw5XoViiMYKYzrp9siNlg59KHltAHonL7zYb4ng1h+VpzJPYgOSwAWwoF7aj8b4Dhfx6MrklcU2apzV7hGBKJsrHhTZ3ad5FXRs0W7iCC3/sOXqq6IUt6bh/uidc/D/DHTBMXlAxDl2h/H5r1ryd4hFmzYpqwQ8tf/j6jmd+OqvXXS4yy1oMIHp86lFZsZKq3n/Zr92ik1akcyYyw+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        chassis.initChassis(this, chassis.CHASSIS_LEFT_FWD,"mtrLeftFront",
                "mtrLeftBack", "mtrRightFront","mtrRightBack",
                "navx",10d,1d,1d,1d,
                1d,"csFront");
        chassis.chassisDontUseLeftTrigBmpr(true);
        chassis.resetChassisAuton(true);
        launcher.initLauncher(this, "mtrElev", "mtrIntake", "mtrConveyor",
                "mtrLauncher", "srvoLoader");

        // Wait for the game to start (driver presses PLAY)
        // Prompt User
        telemetry.addData("Press Start", " >");
        telemetry.update();
        waitForStart();

        launcher.shootRings(this);

        // chassis.autonpivotTurnNavX(this, 180, 10000);

        //launcher.shootRings(this);
        //telemetry.update();
        //sleep(1000);
        chassis.closeChassis(this);


    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private int find_stack(int timeout) {

        initVuforia();
        initTfod();

        /* Activate Tensor Flow Object Detection. */

        if (tfod != null) {
            tfod.activate();

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2, 1.78);
        }

        long lMarkMilliS = System.currentTimeMillis();
        while (((System.currentTimeMillis() - lMarkMilliS) < timeout) && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel() == "Single") {
                            tfod.shutdown();
                            return 1;
                        } else if (recognition.getLabel() == "Quad") {
                            tfod.shutdown();
                            return 4;
                        }
                    }
                    telemetry.update();

                }
            } else {
                telemetry.addData("tfod is null", "");
                telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        return 0;
    }


}