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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Autonomous test v2", group = "SDR")
//@Disabled
public class A_5159_test_2 extends LinearOpMode {


    /* Declare OpMode members. */
    Chassis_KLNavX_6109_v16a chassis = new Chassis_KLNavX_6109_v16a();   // Use Omni-Directional drive system
    Launcher_5159_v1 launcher = new Launcher_5159_v1();   // Use all other motors

    int factor;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    int stack_size = 0;




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
        /*
        chassis.autonpivotTurnNavX(this, 90d, 10000);
        sleep(500);
        chassis.getNavx(this);
        sleep(500);

        chassis.autonpivotTurnNavX(this, 180d, 10000);
        sleep(500);
        chassis.getNavx(this);

         */

        //chassis.autonpivotTurnNavX(this, -90d, 10000);
        //chassis.autonpivotTurnNavX(this, 0d, 10000);


        /*
        double dYaw = chassis.getNavXYaw();
        if (dYaw > 0.5d) {
            chassis.autonpivotTurnNavX(this, -1d, 10000);
        } else if (dYaw < -0.5d) {
            chassis.autonpivotTurnNavX(this, 1d, 10000);
        }

         */

        chassis.autonpivotTurnNavX(this, 175d, 10000);
/*
        dYaw = chassis.getNavXYaw();

        if (dYaw <= 180d && dYaw >= 0d) {
            chassis.autonpivotTurnNavX(this, 179d, 10000);
        } else {
            chassis.autonpivotTurnNavX(this, -179d, 10000);
        }

 */




        //chassis.getNavx(this);





        sleep(3000);
        //chassis.autonmoveNavX(this, 1000, 10000, 0, 0, 0.2);

        //chassis.autonTurnNoNavx(this, 0.2, 1475, 10000);



        // chassis.autonpivotTurnNavX(this, 180, 10000);

        //launcher.shootRings(this);
        //telemetry.update();
        //sleep(1000);
        chassis.closeChassis(this);


    }

//ALL THIS CODE IS TRASH :(
}