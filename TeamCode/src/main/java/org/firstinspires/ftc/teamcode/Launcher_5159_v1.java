 package org.firstinspires.ftc.teamcode;

 import android.graphics.Color;
 import android.util.Log;


 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Gamepad;
 import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
 import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
 import com.qualcomm.robotcore.hardware.NormalizedRGBA;
 import com.qualcomm.robotcore.hardware.SwitchableLight;


public class Launcher_5159_v1 {

     public DcMotor mtrConveyor = null;
     public DcMotor mtrLauncher = null;
     public DcMotor mtrIntake = null;
     public DcMotor mtrElev = null;

    //Servos
     public CRServo srvoLoader = null;

     //motor constants
     double dPwrIntake = 0.7;
     double dPwrConveyor = -0.9;
     double dPwrLauncherPowerShot = -0.61;
     double dPwrLauncherMidGoal = -0.68;
     double dPwrLauncherTopGoalFirst = -0.641;
     double dPwrLauncherTopGoal = -0.643;
     double dPwrElevUp = 0.4;
     double dPwrElevDown = -0.2;

     //servo constants
    double dPwrSrvo = 1;

    //calc RPM Constants
    double previousLauncherPosition = 0;
    double previousTime = 0;
    double RPM = 0;
    double launcherPosition = 0;
    double currentTime = 0;

    public Launcher_5159_v1() {

    }

    public void initLauncher(OpMode opMode, String strElevName,
                             String strIntakeName, String strConveyorName, String strLauncherName, String strServo) {
        //elevator
        mtrElev = opMode.hardwareMap.get(DcMotor.class, strElevName);
        mtrElev.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
        //intake
        mtrIntake = opMode.hardwareMap.get(DcMotor.class, strIntakeName);
        mtrIntake.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
        //conveyor
        mtrConveyor = opMode.hardwareMap.get(DcMotor.class, strConveyorName);
        mtrConveyor.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //launcher
        mtrLauncher = opMode.hardwareMap.get(DcMotor.class, strLauncherName);
        mtrLauncher.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
        mtrLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servos
        srvoLoader=opMode.hardwareMap.get(CRServo.class, strServo);
        srvoLoader.setDirection(CRServo.Direction.FORWARD);

        mtrElev.setPower(0);
        mtrIntake.setPower(0);
        mtrConveyor.setPower(0);
        mtrLauncher.setPower(0);


    }

     //elevator
     public void autonMoveElev(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();
         //Elevator Motor (encoder)
         mtrElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrElev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         mtrElev.setPower(speed);

         lMarkMilliS=System.currentTimeMillis() + timeout;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
                 && Math.abs(mtrElev.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(  msg + " Elev pos:", "%d",
                     mtrElev.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrElev.setPower(0);
     }

     //intake
     public void autonMoveIntake(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();
         //Intake Motor (encoder)
         mtrIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         mtrIntake.setPower(speed);

         lMarkMilliS=System.currentTimeMillis() + timeout;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
                 && Math.abs(mtrIntake.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(  msg + " Intake pos:", "%d",
                     mtrIntake.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrIntake.setPower(0);
     }

     /* ----------------------- */
     //conveyor
     public void autonMoveConveyor(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();
         //Launcher Motor (encoder)
         //mtrConveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //mtrConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         mtrConveyor.setPower(speed);

         lMarkMilliS=System.currentTimeMillis() + timeout;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
                 && Math.abs(mtrConveyor.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(  msg + " Intake pos:", "%d",
                     mtrConveyor.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrConveyor.setPower(0);
     }

     //launcher
     public void autonMoveLauncher(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();
         //Launcher Motor (encoder)
         mtrLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         mtrLauncher.setPower(speed);

         lMarkMilliS=System.currentTimeMillis() + timeout;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
                 && Math.abs(mtrLauncher.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(  msg + " Intake pos:", "%d",
                     mtrLauncher.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrLauncher.setPower(0);
     }

    public void autonMoveServo(LinearOpMode opMode, String msg, double power, int timeout) {
        long lMarkMilliS;
        lMarkMilliS=System.currentTimeMillis() + timeout;
        srvoLoader.setPower(power);
        while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()) {
            opMode.telemetry.addData(  msg + " Srvo pos:", "%.2f",
                    srvoLoader.getPower());
            opMode.telemetry.update();
        }
        srvoLoader.setPower(0);
    }
    // ---------------------------------------------------------------------------------------------------------------------//

     public void shootRings(LinearOpMode opMode) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();

         //Launcher Motor (encoder)
         mtrLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         mtrLauncher.setPower(-0.64);
         lMarkMilliS=System.currentTimeMillis() + 2750;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrConveyor.setPower(dPwrConveyor);

         lMarkMilliS=System.currentTimeMillis() + 600;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrConveyor.setPower(0);
         mtrLauncher.setPower(-0.644);

         lMarkMilliS=System.currentTimeMillis() + 1500;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrConveyor.setPower(dPwrConveyor);
         lMarkMilliS=System.currentTimeMillis() + 750;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }
         mtrIntake.setPower(dPwrIntake);
         mtrLauncher.setPower(-0.64);


         lMarkMilliS=System.currentTimeMillis() + 3500;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }
         mtrLauncher.setPower(0);
         mtrConveyor.setPower(0);
         mtrIntake.setPower(0);
     }

    void telMoveElev(Gamepad gamepad2) {
        //            Y
        //        X       B
        //            A
        //

        //manual control
        if (gamepad2.left_trigger > (0.67)) {
            mtrElev.setPower(dPwrElevDown);
        } else if (gamepad2.left_bumper) {
            mtrElev.setPower(dPwrElevUp);
        } else {
            mtrElev.setPower(0.0);
        }
    }
    //moving the intake
    void telMoveIntake(Gamepad gamepad2) {
        //manual control
        if (gamepad2.dpad_up) {
            mtrIntake.setPower(dPwrIntake);
        } else if (gamepad2.dpad_down) {
            mtrIntake.setPower(-dPwrIntake);
        } else {
            mtrIntake.setPower(0.0);
        }
    }
    //moving the conveyor
    void telMoveConveyor(Gamepad gamepad2) {
        //manual control
        if (gamepad2.a) {
            mtrConveyor.setPower(-dPwrConveyor);
        } else if (gamepad2.y) {
            mtrConveyor.setPower(dPwrConveyor);
        } else {
            mtrConveyor.setPower(0.0);
        }
    }

    //moving the launcher
    void telMoveLauncher(Gamepad gamepad2) {
        //manual control
        if (gamepad2.right_bumper) {
            mtrLauncher.setPower(dPwrLauncherPowerShot);
        } else if (gamepad2.right_trigger > (0.67)) {
            mtrLauncher.setPower(dPwrLauncherTopGoal);
        }
        else {
            mtrLauncher.setPower(0.0);
        }
    }
    //moving the servo
    void telMoveServo(Gamepad gamepad2) {
        // slew the servo, according to the rampUp (direction) variable.
        if (gamepad2.dpad_left) {
            srvoLoader.setPower(-dPwrSrvo);
        } else if (gamepad2.dpad_right) {
            srvoLoader.setPower(dPwrSrvo);
        } else {
            srvoLoader.setPower(0);
        }
    }

    //moving the servo
    public void calcRPM(OpMode opMode) {
         long lMarkMilliS;


        launcherPosition = mtrLauncher.getCurrentPosition();
        currentTime = System.currentTimeMillis();
        if((currentTime-previousTime)<100) return;
        RPM = (launcherPosition-previousLauncherPosition)/(currentTime-previousTime);

        previousLauncherPosition = launcherPosition;
        previousTime = currentTime;

        opMode.telemetry.addData( "Launcher rate:", "%f",
                RPM);
        opMode.telemetry.update();



    }
}
