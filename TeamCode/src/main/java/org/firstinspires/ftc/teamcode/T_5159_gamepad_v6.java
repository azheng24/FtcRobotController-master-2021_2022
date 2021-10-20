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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


  @TeleOp(name="Teleop 5159 Gamepad v6 Hardware", group="SDR")
//@Disabled
public class T_5159_gamepad_v6 extends OpMode {


      /* Declare OpMode members. */
    Chassis_KLNavX_6109_v6 chassis = new Chassis_KLNavX_6109_v6();   // Use Omni-Directional drive system
      Launcher_5159_v1 launcher = new Launcher_5159_v1();   // Use Omni-Directional drive system

      @Override
    public void init() {
        // Initialize the chassis
          chassis.initChassis(this, chassis.DIRECTCONNECT,true );
          chassis.resetMotorsTeleop(true);
          launcher.initLauncher(this, "mtrElev", "mtrIntake", "mtrConveyor",
                  "mtrLauncher", "srvoLoader");


          // Wait for the game to start (driver presses PLAY)
       // Prompt User
        telemetry.addData("Press Start", " >");
        telemetry.update();

      }


      @Override
    public void stop() {

        //SkyStone v5.2 incompatibility
        //if(chassis.isNavXConnected()) chassis.enableNavXYawPID(false);

        chassis.closeNavX();
        telemetry.addData("[]", "Shutting Down. Bye!");
        telemetry.update();
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */

    @Override
    public void init_loop() {
         // telemetry.addData("navX Op Init Loop", runtime.toString());
          //telemetry.update();

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    //moving the elevator



    @Override
    public void loop() {

        // run until the end of the match (driver presses STOP)

        chassis.chkuseTankControl(gamepad1);
        chassis.chkuseOmniControl(gamepad1);
        launcher.telMoveElev(gamepad2);
        launcher.telMoveServo(gamepad2);
        launcher.telMoveIntake(gamepad2);
        launcher.telMoveConveyor(gamepad2);
        launcher.telMoveLauncher(gamepad2);
        launcher.calcRPM(this);


        //function: check rpm
        //time
        /*

        -time
        -ecnoder count
        -previous times
        -subtract
        -store

        target rpm

        having a ring could be different

       shoot dunction
       is this rpm in the range
         */
        //previous time and previous encoder count to find rpm


        //UNCOMPATIBLE
        /*
        //send telemetry to driver station (no robot control)
        if (chassis.getControlMode() == chassis.OMNI_CONTROL) {
            telemetry.addData("Omni Mode ", "Direction Stick Polar%.2f",
            chassis.getDirectionStickPolarAngle());
        } else if (chassis.getControlMode() == chassis.TANK_CONTROL) {
            telemetry.addData("Tank Mode ", "LeftStick Y:%.2f RightStick Y:%.2f",
                gamepad1.left_stick_y, gamepad1.right_stick_y);
            telemetry.addData("      ", "%s",chassis.getTankMovement());
        }
        telemetry.addData("NavX:","%s",chassis.getNavXStatus());
        if(chassis.isNavXConnected()) {
            telemetry.addData("YAW ", "Yaw to Hold%.2f YawPwr:%.2f",
                chassis.getYawAngleToHold(), chassis.getYawPwr());
            telemetry.addData("Heading ", "%.2f",
                chassis.getNavXYaw());
        } else  {
            telemetry.addData("WARNING! ","NavX not connected!i!");
        }
        telemetry.addData("PWR", "LF:%.2f RF:%.2f",
            chassis.getResultantLFPwr(), chassis.getResultantRFPwr());
        telemetry.addData("   ", "LB:%.2f RB:%.2f",
            chassis.getResultantLBPwr(), chassis.getResultantRBPwr());
        telemetry.addData("MTR ", " LF %.2f RF %.2f",
            chassis.mtrLeftFront.getPower(), chassis.mtrRightFront.getPower());
        telemetry.addData("    ", " LB %.2f RB %.2f",
            chassis.mtrLeftBack.getPower(), chassis.mtrRightBack.getPower());
        telemetry.update();

         */


    }



}