package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;




import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//SkyStone V5.2 does not seem to take the modifications to teamcode/build.release.gradle
//so cannot use below
//import com.kauailabs.navx.ftc.AHRS;
//import com.kauailabs.navx.ftc.navXPIDController;
//The Kauai Labs Library calls are commented throughout code below,
// in the hope that future releases will be compatible


/**
 * This is NOT an opmode... the code is used by an opmode.  See T_SDR_v5s.java for a sample opmode
 * that uses the class below
 *
 * The class uses the following names in your robot configuration:
 *
 * mtrLeftFront
 * mtrLeftBack
 * mtrRightFront
 * mtrRightBAck
 * navx
 *
 * FTC Teams 6109 and 6963
 *
 *
 */


public class Chassis_KLNavX_6109_v6 {


    //For those using a Modern Robotics Device Interface Module
    // This is the port on the Core Device Interface Module        */
    // in which the navX-Model Device is connected.  Modify this  */

    private final int NAVX_DIM_I2C_PORT = 0;
    //SkyStone v5.2 incompatibility
    //private AHRS navx_device;
    //private navXPIDController yawPIDController;
    //private ElapsedTime runtime = new ElapsedTime();
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    //private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    //public final double YAW_PID_P = 0.005; //original
    private final double YAW_PID_P = 0.01;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    private final double CCSCALAR = .0025; //.003;
    private final double CCCAP = .4; //.66

    private boolean calibration_complete = false;

    //For those using a Rev Hub without a MR Device Interface Module
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    //Motors
    public DcMotor mtrLeftBack = null;
    public DcMotor mtrRightBack = null;
    public DcMotor mtrLeftFront = null;
    public DcMotor mtrRightFront = null;
    /*
    public DcMotor mtrElev = null;
    public DcMotor mtrIntake = null;
    public DcMotor mtrConveyor = null;
    public DcMotor mtrLauncher = null;
    */

    ColorSensor colorSensor;    // Hardware Device Object


    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    // bLedOn represents the state of the LED.
    boolean bLedOn;
    double last_hue = 0;


    // public CRServo srvoLoader = null;
    //public static final double SERVOLOADERTRANSPORT = 0.99;
    //public static final double SERVOLOADERLOAD = -0.99;//.5 is flat
    //public static final double SERVOLOADERDUMP = 0.;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  position = (0.1); // Start at halfway position
    //boolean rampUp = true;

    public final int MOTORSTURNED = 1;
    public final int DIRECTCONNECT = 0;


    //Public constants
    public final int OMNI_CONTROL = 1;
    public final int TANK_CONTROL = 2;
    public final int START_FAST = 1;
    public final int START_SLOW = 2;
    public final int START_FWD = 1;
    public final int START_REV = 2;
    public final int START_RIGHT_HAND = 1;
    public final int START_LEFT_HAND = 2;

    //Tank drive constants
    private final int TANK_FASTFWD = 100;
    private final int TANK_FASTREV = -100;
    private final int TANK_SLOWFWD = 1;
    private final int TANK_SLOWREV = -1;
    private final double TANK_FAST_PWR = 0.38;
    private final double TANK_SLOW_PWR = .3;
    private final int TANK_NO_MOVEMENT = 0;
    private final int TANK_MOVEMENT = 1;
    private final int TANK_LATERAL_MOVEMENT = 2;
    private final float TANK_STICK_DEADZONE = (float) .67;
    //Tank drive global variables
    private double dTankSpeedDir = TANK_FASTFWD;
    private int nTankMovement = TANK_NO_MOVEMENT;
    private int nTankPrevMove = TANK_NO_MOVEMENT;

    //Omni drive constants
    private final double OMNI_FAST_PWR = 1.0;
    private final double OMNI_SLOW_PWR = .5;
    private final float OMNI_STICK_DEADZONE = (float) .33;
    private final boolean STICK_RIGHT = false, STICK_LEFT = true;
    private final double STICK_IN_DEADZONE = 99999.99; //not allowed polar angle
    private final int CCW = -1, CW = 1, NOSPIN = 0;
    //Omni drive global variables/
    private boolean bYawPressed = false; //previous read of yaw stick
    private double dOmniFullPwr = OMNI_FAST_PWR;
    private double dRobotDirectionPolar = 0;
    private double dDirectionStickPolar;
    private boolean bYawStick = STICK_LEFT, bDirectionStick = STICK_RIGHT;
    private double dMomentumAdjustment = (double) 5.0;
    private int nRobotSpin;


    //Common constants
    private final double YAW_DIR_SCALAR = (double) .5;
    private final double FAST_SPIN_ADJ = (double) 11, SLOW_SPIN_ADJ = (double) 5;
    private final int NAVX_NONE = 0, NAVX_AS_YAWPID = 1, NAVX_AS_GYRO = 2;

    //Common global variables
    private int nNavXStatus = NAVX_NONE;
    private double dYawAngleToHold = 0;
    private double dYawPwr; //power for each wheel to get desired yaw to hold
    private double //power for each wheel from direction controls
            dDirLFPwr = 0, dDirRFPwr = 0,
            dDirLBPwr = 0, dDirRBPwr = 0;
    private double //power for each wheel adding yaw and direction, then scaling the largest to 1
            dResultantLFPwr = 0, dResultantRFPwr = 0,
            dResultantLBPwr = 0, dResultantRBPwr = 0;

    private int nControlMode = TANK_CONTROL;
    private boolean bHasMecanum;

    private long lBackTimestamp = System.currentTimeMillis(); //button timestamp
    private long lLastControlPress = System.currentTimeMillis(); //control timestamp

    /* Constructor */
    public Chassis_KLNavX_6109_v6() {

    }


    /* Initialize standard Hardware interfaces */
    public void initChassis(OpMode opMode, int nChassisLayout, boolean bUseMecanum) {
        //setup nav
        bHasMecanum = bUseMecanum;

        initNavX(opMode);
        initMotors(opMode, nChassisLayout, bUseMecanum);
    }

    public void initMotors(OpMode opMode, int nReqLayout, boolean bUseMecnum) {
        /*
        //elevator
        mtrElev = opMode.hardwareMap.get(DcMotor.class, "mtrElev");
        mtrElev.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //intake
        mtrIntake = opMode.hardwareMap.get(DcMotor.class, "mtrIntake");
        mtrIntake.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //conveyor
        mtrConveyor = opMode.hardwareMap.get(DcMotor.class, "mtrConveyor");
        mtrConveyor.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //launcher
        mtrLauncher = opMode.hardwareMap.get(DcMotor.class, "mtrLauncher");
        mtrLauncher.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

        //servos
        srvoLoader=opMode.hardwareMap.get(CRServo.class, "srvoLoader");
        srvoLoader.setDirection(CRServo.Direction.FORWARD);
*/
        //setup motors
        // Define and Initialize Motors
        mtrLeftBack = opMode.hardwareMap.get(DcMotor.class, "mtrLeftBack");
        mtrRightBack = opMode.hardwareMap.get(DcMotor.class, "mtrRightBack");
        /*
        mtrElev.setPower(0);
        mtrIntake.setPower(0);
        mtrConveyor.setPower(0);
        mtrLauncher.setPower(0);

         */

        if (bUseMecnum) {
            mtrLeftFront = opMode.hardwareMap.get(DcMotor.class, "mtrLeftFront");
            mtrRightFront = opMode.hardwareMap.get(DcMotor.class, "mtrRightFront");
        }
        if (nReqLayout == MOTORSTURNED) {
            if (bUseMecnum) {
                mtrLeftFront.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
                mtrRightFront.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates counter clockwise
            }
            mtrLeftBack.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
            mtrRightBack.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates counter clockwise
        } else if (nReqLayout == DIRECTCONNECT) {
            if (bUseMecnum) {
                mtrLeftFront.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
                mtrRightFront.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            }
            mtrLeftBack.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
            mtrRightBack.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
        } else {
            if (bUseMecnum) {
                mtrLeftFront.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
                mtrRightFront.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
            }
            mtrRightBack.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
            mtrLeftBack.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
        }

        // Stop all chassis motion by setting each axis value to zero
        if (bUseMecnum) {
            mtrLeftFront.setPower(0);
            mtrRightFront.setPower(0);
        }
        mtrLeftBack.setPower(0);
        mtrRightBack.setPower(0);
    }

    public void initNavX(OpMode opMode) {
        //setup nav
        opMode.telemetry.addData("Status:", "Setting up navx");
        opMode.telemetry.update();

        //SkyStone v5.2 incompatibility
        /*
        if(calibrateNavXYawPID(opMode)==true) {
            nNavXStatus = NAVX_AS_YAWPID;
        } else */
        if (calibrateNavXAsGyro(opMode) == true) {
            nNavXStatus = NAVX_AS_GYRO;
        } else {
            nNavXStatus = NAVX_NONE;
        }
    }

    public boolean chkuseTankControl(Gamepad gamepad) {
        //here's the dpad and button pad:
        //
        // dpad                         buttons
        //          Up                        Y (normal)
        //      Left  Right             X(slow)  B(fast)
        //        Down                        A (reverse)
        //
        //if the driver hits a button on the button pad (Y,X,B,A) the robot goes into tank drive
        //   if the Y button is hit, tank drive is normal
        //       left stick forward/back sets both left wheels power to forward/back
        //       right stick forward/back sets both right wheel power to forward/back
        //       both sticks left moves the robot left     LF wheel back    RF wheel fwd
        //                                                 LB wheel fwd     RB wheel back
        //       both sticks right moves the robot right   LF wheel fwd     RF wheel back
        //                                                 LB wheel back    RB wheel fwd
        //
        //   if the A button is hit, tank drive is reversed (this makes the "front" of the robot the "back")
        //       this mode is useful when have to drive the robot backwards
        //       left stick forward/back sets both right wheels power to back/forward
        //       right stick forward/back sets both left wheels power to back/forward
        //       both sticks left moves the robot right    LF wheel fwd    RF wheel back
        //                                                 LB wheel right  RB wheel fwd
        //
        //  if the X button is hit robot power is slow
        //      apply a slow power when the sticks are moved
        //
        //  if the B button is hit robot power is fast
        //      apply a fast power when the sticks are moved
        //

        if ((System.currentTimeMillis() - lLastControlPress) < 250) return false;

        if (gamepad.x) {//hit x, slow speed setting
            if (dTankSpeedDir == TANK_FASTFWD)
                dTankSpeedDir = TANK_SLOWFWD;
            else if (dTankSpeedDir == TANK_FASTREV)
                dTankSpeedDir = TANK_SLOWREV;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                dTankSpeedDir = TANK_SLOWFWD;
            }
            nTankMovement = TANK_NO_MOVEMENT;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                gofromOmniToTank();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in tank control
        } else if (gamepad.b) {//hit b, fast speed setting
            if (dTankSpeedDir == TANK_SLOWFWD)
                dTankSpeedDir = TANK_FASTFWD;
            else if (dTankSpeedDir == TANK_SLOWREV)
                dTankSpeedDir = TANK_FASTREV;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                dTankSpeedDir = TANK_FASTFWD;
            }
            nTankMovement = TANK_NO_MOVEMENT;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                gofromOmniToTank();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in tank control
        } else if (gamepad.y) {//hit y,fwd
            if (dTankSpeedDir == TANK_SLOWREV)
                dTankSpeedDir = TANK_SLOWFWD;
            else if (dTankSpeedDir == TANK_FASTREV)
                dTankSpeedDir = TANK_FASTFWD;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                dTankSpeedDir = TANK_FASTFWD;
            }
            nTankMovement = TANK_NO_MOVEMENT;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                gofromOmniToTank();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in tank control
        }  else if (gamepad.a) {//hit a, invert
            if (dTankSpeedDir == TANK_SLOWFWD)
                dTankSpeedDir = TANK_SLOWREV;
            else if (dTankSpeedDir == TANK_FASTFWD)
                dTankSpeedDir = TANK_FASTREV;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                dTankSpeedDir = TANK_FASTREV;
            }
            nTankMovement = TANK_NO_MOVEMENT;
            if (nControlMode != TANK_CONTROL) {
                nControlMode = TANK_CONTROL;
                gofromOmniToTank();
            }

            lLastControlPress = System.currentTimeMillis();
            return true; //now in tank control
        }

        //if no button hit got here
        //check if we are in TANK CONTROL
        if (nControlMode != TANK_CONTROL)
            return false; //no button hit not in tank control, go back

        //if got here, in tank control so can set power
        setTankPwr(gamepad);
        if (bHasMecanum) {
            mtrLeftFront.setPower(dResultantLFPwr);
            mtrRightFront.setPower(dResultantRFPwr);
        }
        mtrLeftBack.setPower(dResultantLBPwr);
        mtrRightBack.setPower(dResultantRBPwr);
        return true;
    }

    private void gofromOmniToTank() {
        nTankMovement = TANK_NO_MOVEMENT;
        nTankPrevMove = TANK_NO_MOVEMENT;
        setMecWheelDirectionPwrZero();
        dYawPwr = 0;
        bYawPressed = false;
        nRobotSpin = NOSPIN;
        //SkyStone v5.2 incompatibility
        //enableNavXYawPID(true);
        setpointNavXYaw(getNavXYaw());
    }


    private void setTankPwr(Gamepad gamepad) {
        int nRes;

        //find power to apply to each wheel from the stick positions
        nTankPrevMove = nTankMovement;
        nTankMovement = setTankDirPwr(gamepad);
        //find the yaw power to apply to each wheel to correct lateral movement
        setTankYawPwr(nTankPrevMove, nTankMovement);
        //at this point:
        // 1.  Have the power to apply to each wheel to go in the stick's direction
        //     while maintaining existing yaw
        //     The power to apply stored in dDirLFPwr, dDirFRPwr, etc)
        // 2.  Have the yaw power to rotate the robot (dYawPwr), that adds/subs from existing yaw
        //Adding yaw power to each wheel's directional  power will result in a simultaneous spin and move
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another
        addDirAndYawPwr();
    }

    private int setTankDirPwr(Gamepad gamepad) {

        boolean bFoundLeftInput = false, bFoundRightInput = false;
        int nRC = TANK_NO_MOVEMENT;
        double dPwr;
        if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_FASTREV))
            dPwr = TANK_FAST_PWR;
        else //slow power required
            dPwr = TANK_SLOW_PWR;

        //now apply power
        //check left y
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +

        //check left stick up
        if ((gamepad.left_stick_y < -TANK_STICK_DEADZONE) &&  //left stick moved UP!!
                (gamepad.left_stick_x < (TANK_STICK_DEADZONE / (float) 1)) &&
                (gamepad.left_stick_x > -(TANK_STICK_DEADZONE / (float) 1))) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirLFPwr = dPwr; //forward
                dDirLBPwr = dPwr; //forward
                //mtrLeftBack.setPower(dPwr);  //forward
                //mtrLeftFront.setPower(dPwr); //forward
            } else { //chassis is reversed
                dDirRFPwr = -dPwr; //backward
                dDirRBPwr = -dPwr; //backward
                //mtrRightFront.setPower(-dPwr); //backward
                //mtrRightBack.setPower(-dPwr);  //backward
            }
            nRC = TANK_MOVEMENT;
            bFoundLeftInput = true;
        }
        //check left stick down
        if ((gamepad.left_stick_y > TANK_STICK_DEADZONE) &&  //left stick moved DOWN!!
                (gamepad.left_stick_x < (TANK_STICK_DEADZONE / (float) 1)) &&
                (gamepad.left_stick_x > -(TANK_STICK_DEADZONE / (float) 1))) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirLFPwr = -dPwr; //backward
                dDirLBPwr = -dPwr; //backward
                //mtrLeftFront.setPower(-dPwr); //backward
                //mtrLeftBack.setPower(-dPwr);  //backward
            } else {  //chassis is reversed
                dDirRFPwr = dPwr; //forward
                dDirRBPwr = dPwr; //forward
                //mtrRightFront.setPower(dPwr); //forward
                //mtrRightBack.setPower(dPwr);  //forward
            }
            nRC = TANK_MOVEMENT;
            bFoundLeftInput = true;
        }
        //check right stick up
        if ((gamepad.right_stick_y < -TANK_STICK_DEADZONE) && //right stick moved UP!!
                (gamepad.right_stick_x < (TANK_STICK_DEADZONE / (float) 1)) &&
                (gamepad.right_stick_x > -(TANK_STICK_DEADZONE / (float) 1))) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirRFPwr = dPwr; //forward
                dDirRBPwr = dPwr; //forward
                //mtrRightFront.setPower(dPwr);  //forward
                //mtrRightBack.setPower(dPwr);  //forward
            } else { //chassis is reversed
                dDirLFPwr = -dPwr; //backward
                dDirLBPwr = -dPwr; //backward
                //mtrLeftFront.setPower(-dPwr);  //backward
                //mtrLeftBack.setPower(-dPwr);  //backward
            }
            nRC = TANK_MOVEMENT;
            bFoundRightInput = true;
        }
        //check right stick down
        if ((gamepad.right_stick_y > TANK_STICK_DEADZONE) &&  //right stick moved DOWN
                (gamepad.right_stick_x < (TANK_STICK_DEADZONE / (float) 1)) &&
                (gamepad.right_stick_x > -(TANK_STICK_DEADZONE / (float) 1))) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirRFPwr = -dPwr; //backward
                dDirRBPwr = -dPwr; //backward
                //mtrRightFront.setPower(-dPwr);  //backward
                //mtrRightBack.setPower(-dPwr);  //backward
            } else { //chassis is reversed
                dDirLFPwr = dPwr; //forward
                dDirLBPwr = dPwr; //forward
                dYawPwr = 0; //no correction for forward/backward
                //mtrLeftFront.setPower(dPwr);  //forward
                //mtrLeftBack.setPower(dPwr);  //forward
            }
            nRC = TANK_MOVEMENT;
            bFoundRightInput = true;
        }

        //now check left-right lateral movement
        //do this only if have mecanum whheels
        if (bHasMecanum) {
            //check for move left  (both sticks left)
            if ((gamepad.left_stick_x < -TANK_STICK_DEADZONE) &&  //left stick moved LEFT
                    (gamepad.left_stick_y > -(TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.left_stick_y < (TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.right_stick_x < -TANK_STICK_DEADZONE) &&  //right stick moved LEFT
                    (gamepad.right_stick_y > -(TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.right_stick_y < (TANK_STICK_DEADZONE / (float) 1))) {
                if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) { //move left
                    dDirLFPwr = -dPwr;
                    dDirRFPwr = dPwr;
                    dDirLBPwr = dPwr;
                    dDirRBPwr = -dPwr;
                } else { //chassis is reversed so move right
                    dDirLFPwr = dPwr;
                    dDirRFPwr = -dPwr;
                    dDirLBPwr = -dPwr;
                    dDirRBPwr = dPwr;
                }
                nRC = TANK_LATERAL_MOVEMENT;
                bFoundLeftInput = true;
                bFoundRightInput = true;
            }

            //check for move right (both sticks right)
            if ((gamepad.left_stick_x > TANK_STICK_DEADZONE) &&  //left stick moved RIGHT
                    (gamepad.left_stick_y > -(TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.left_stick_y < (TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.right_stick_x > TANK_STICK_DEADZONE) &&  //right stick moved RIGHT
                    (gamepad.right_stick_y > -(TANK_STICK_DEADZONE / (float) 1)) &&
                    (gamepad.right_stick_y < (TANK_STICK_DEADZONE / (float) 1))) {
                if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) { //move right
                    dDirLFPwr = dPwr;
                    dDirRFPwr = -dPwr;
                    dDirLBPwr = -dPwr;
                    dDirRBPwr = dPwr;
                } else { //chassis is reversed so move left
                    dDirLFPwr = -dPwr;
                    dDirRFPwr = dPwr;
                    dDirLBPwr = dPwr;
                    dDirRBPwr = -dPwr;
                }
                nRC = TANK_LATERAL_MOVEMENT;
                bFoundLeftInput = true;
                bFoundRightInput = true;
            }
        }

        //now check if sticks were moved.  turn off motors it they were not
        if (!bFoundLeftInput) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirLFPwr = 0;
                dDirLBPwr = 0;
            } else { //chassis is reversed
                dDirRFPwr = 0;
                dDirRBPwr = 0;
            }
        }
        if (!bFoundRightInput) {
            if ((dTankSpeedDir == TANK_FASTFWD) || (dTankSpeedDir == TANK_SLOWFWD)) {
                dDirRFPwr = 0;
                dDirRBPwr = 0;
            } else {
                dDirLFPwr = 0;
                dDirLBPwr = 0;
            }
        }
        return nRC;

    }

    private void setTankYawPwr(int nPrev, int nCurrent) {

        if (nCurrent != TANK_LATERAL_MOVEMENT) {
            dYawPwr = 0;
            return;
        }
        if (!isNavXConnected()) {
            dYawPwr = 0;
            return;
        }

        //set target heading the when the driver starts  lateral movement;
        if (((nPrev != TANK_LATERAL_MOVEMENT) && (nCurrent == TANK_LATERAL_MOVEMENT)) /*||
            ((nPrev!=TANK_MOVEMENT)&&(nCurrent==TANK_LATERAL_MOVEMENT))*/) {
            //if got here lateral motion was just started, set direction to keep
            setpointNavXYaw(getNavXYaw());

        }

        dYawPwr = getNavXCorrectionPwr();
    }

    public boolean chkuseOmniControl(Gamepad gamepad) {
        //here's the dpad and button pad:
        //
        // dpad                                     buttons
        //              Up(normal)                      Y
        //      Left(slow)  Right(fast)             X       B
        //            Down(reverse)                     A

        //if the driver hits any of the dpad switches (up,down,left,right),the robot is put in omni drive
        //
        //   if the up switch is hit omni is drive normal
        //      left stick controls robot yaw,
        //      right stick controls robot's field oriented direction
        //
        //   if the down switch is hit  omni drive is reverse NOT YET IMPLEMENTED
        //      right stick controls robot yaw,
        //      left stick controls robot's field oriented direction
        //
        //   if the left switch is hit
        //      apply a slow power when the sticks are moved
        //
        //   if the right switch is hit
        //      apply a fast power when the sticks are moved
        //

        if ((System.currentTimeMillis() - lLastControlPress) < 250) return false;
        if (!bHasMecanum) return false; //cant do this witout mecanum

        if (gamepad.dpad_left) {//hit left , slow speed setting
            //normally power is derived from the magnitude of the stick throw
            //but since using thumb sticks, get power from dpad
            dOmniFullPwr = OMNI_SLOW_PWR;
            dMomentumAdjustment = SLOW_SPIN_ADJ;
            //check if this is a control mode change
            if (nControlMode != OMNI_CONTROL) {
                nControlMode = OMNI_CONTROL; //hit dpad, so automatically in field orientation
                gofromTankToOmni();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in omni control
        } else if (gamepad.dpad_right) {//hit right , fast speed setting
            //normally power is derived from the magnitude of the stick throw
            //but since using thumb sticks, get power from dpad
            dOmniFullPwr = OMNI_FAST_PWR;
            dMomentumAdjustment = FAST_SPIN_ADJ;
            //check if this is a control mode change
            if (nControlMode != OMNI_CONTROL) {
                nControlMode = OMNI_CONTROL; //hit dpad, so automatically in field orientation
                gofromTankToOmni();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in omni control
        } else if (gamepad.dpad_up) {//hit top, "normal", so right stick is direction, left is yaw
            //"right hand" mode
            bDirectionStick = STICK_RIGHT;
            bYawStick = STICK_LEFT;
            //check if this is a control mode change
            if (nControlMode != OMNI_CONTROL) {
                nControlMode = OMNI_CONTROL; //hit dpad, so automatically in field orientation
                gofromTankToOmni();
            }
            lLastControlPress = System.currentTimeMillis();
            return true; //now in omni control
        } else if (gamepad.dpad_down) {//hit down, "invert", so left stick is direction, right is yaw
            //"left hand" mode
            bDirectionStick = STICK_LEFT;
            bYawStick = STICK_RIGHT;
            //check if this is a control mode change
            if (nControlMode != OMNI_CONTROL) {
                nControlMode = OMNI_CONTROL; //hit dpad, so automatically in field orientation
                gofromTankToOmni();
            }

            lLastControlPress = System.currentTimeMillis();
            return true; //now in omni control
        }

        //if no dpad pressed if got here
        //check if we are already in OMNI CONTROL
        if (nControlMode != OMNI_CONTROL)
            return false; //no dpad hit, not in field oriented, go back

        //if got here, in omni control, so look sticks to power each wheel
        setOmniPwr(gamepad, bDirectionStick, bYawStick);

        mtrLeftFront.setPower(dResultantLFPwr);
        mtrRightFront.setPower(dResultantRFPwr);
        mtrLeftBack.setPower(dResultantLBPwr);
        mtrRightBack.setPower(dResultantRBPwr);

        return true;
    }

    private void gofromTankToOmni() {
        nControlMode = OMNI_CONTROL; //hit dpad, so automatically in field orientation
        nTankMovement = TANK_NO_MOVEMENT;
        nTankPrevMove = TANK_NO_MOVEMENT;
        setMecWheelDirectionPwrZero();
        dYawPwr = 0;
        bYawPressed = false;
        nRobotSpin = NOSPIN;

        //SkyStone v5.2 incompatibility
        /*
        enableNavXYawPID(true);
         */
        setpointNavXYaw(getNavXYaw());
    }


    private void setOmniPwr(Gamepad gamepad, boolean bDirStick, boolean bYawStick) {

        //find power to apply to each wheel from the direction stick position
        setOmniDirPwr(gamepad, bDirStick);
        //find the yaw power to apply to each wheel from the yaw stick position
        setOmniYawPwr(gamepad, bYawStick);
        //at this point:
        // 1.  Have the power to apply to each wheel to go in the direction stick direction
        //     while maintaining existing yaw
        //     This direction stick power to apply stored in dDirLFPwr, dDirFRPwr, etc)
        // 2.  Have the yaw power to rotate the robot (dYawPwr), that adds/subs from existing yaw
        //Adding yaw power to each wheel's directional  power will result in a simultaneous spin and move
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another
        addDirAndYawPwr();
    }

    private void addDirAndYawPwr() {
        //Adding yaw power to each wheel's directional  power will result in a simultaneous spin and move
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another.  In this case, the highest wheel power will be 1.
        int LF = 0, RF = 1, LB = 2, RB = 3;
        int nIdx;

        double adResultantPwr[];
        adResultantPwr = new double[4];
        double dHighestPwr, dLowestPwr, dScalar;

        adResultantPwr[LF] = dDirLFPwr + dYawPwr;
        adResultantPwr[RF] = dDirRFPwr - dYawPwr;
        adResultantPwr[LB] = dDirLBPwr + dYawPwr;
        adResultantPwr[RB] = dDirRBPwr - dYawPwr;

        dHighestPwr = adResultantPwr[0];
        dLowestPwr = adResultantPwr[0];
        //find highest and lowest valuej
        for (nIdx = 0; nIdx < 4; nIdx++)
            if (adResultantPwr[nIdx] > dHighestPwr) dHighestPwr = adResultantPwr[nIdx];
            else if (adResultantPwr[nIdx] < dLowestPwr)
                dLowestPwr = adResultantPwr[nIdx];

        //get the largest magnitude, store in dScalar:
        if (Math.abs(dHighestPwr) > Math.abs(dLowestPwr)) dScalar = Math.abs(dHighestPwr);
        else dScalar = Math.abs(dLowestPwr);

        //see if sum of yaw and direction (stored in dScalar) is greater than 1
        //  if the scalsr is greater than 1, need to scale all the wheel's power down until
        //  the point that the highest power is 1, since can't setpower() to more than 1
        if (dScalar > (double) 1) {
            //if got here the sum of yaw and direction is too large
            //must scale all values down by the same amount
            for (nIdx = 0; nIdx < 4; nIdx++) {
                adResultantPwr[nIdx] = adResultantPwr[nIdx] / dScalar;
            }
        }

        dResultantLFPwr = adResultantPwr[LF];
        dResultantRFPwr = adResultantPwr[RF];
        dResultantLBPwr = adResultantPwr[LB];
        dResultantRBPwr = adResultantPwr[RB];
    }

    private void setOmniDirPwr(Gamepad gamepad, boolean bDirStick) {
        // Set the power to apply to each wheel to go in desired direction
        //     global variables set: dDirLFPwr, dDirLBPwr, dDirRFPwr, dDirRBPwr
        // Use the direction stick to get the desired direction
        // Note that the robot may not be pointed forward, but must still go in the stick direction
        //
        // The direction stick works like this:
        //    if not in dead zone
        //       use x and y stick coordinates to calc polar angle to move robot
        //       move robot in direction of the stick, but recall that the front of the robot may not be pointed forward
        //       this means the robot may be turned, but must still move in direction of direction stick
        //    if in dead zone
        //       no input, so don't move robot fwd, back, left or right
        //       set robot direction polar angle to 0 and return false (0 angle should not be used)
        //       robot will not move in any direction, but outside of this method robot can still spin
        //
        double dPolar;

        //first get the polar angle of the direction stick
        if ((dPolar = setDirectionStickPolar(gamepad, bDirStick)) != STICK_IN_DEADZONE) {
            //If got here got an input from the direction stick (not in dead zone)
            dDirectionStickPolar = dPolar;
            //Add the robot yaw angle to the direction stick angle, store in dRobotDirectionPolar
            //use the imu Yaw and the dDirectionStickPolar found above
            dRobotDirectionPolar = setRobotDirectionPolar();
            //From the robot's perspective, moving in dRobotDirectionPolar direction will
            //move the robot in the stick direction, while keeping its yaw (the robot may not have been pointing straight fwd)
            //set the mec wheel power to move in this direction
            //note that this calc is for the robot's existing yaw, not considering the yaw stick yet
            setMecWheelDirPwr(dRobotDirectionPolar);
        } else { //no input from the direction stick (in dead zone)
            //do not apply direction power to wheels, because no direction stick position
            setMecWheelDirectionPwrZero();
        }
    }

    private double setDirectionStickPolar(Gamepad gamepad, boolean bDirStick) {
        double dStickX, dStickY;
        double dAngle, dPolar;


        //calculate and correct stick's polar angle

        if (chkStickDeadZone(gamepad, bDirStick)) {
            //do not move robot in any direction from the direction stick
            //   robot may still spin from the yaw output
            dDirectionStickPolar = 0;
            return STICK_IN_DEADZONE;
        }

        //if got here, the direction stick is pressed
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
        if (bDirStick == STICK_LEFT) {
            //check if stick is in dead zone
            dStickX = (double) gamepad.left_stick_x;
            if (dStickX == 0) dStickX = .000001; //so don't divide by 0
            dStickY = -(double) gamepad.left_stick_y;
        } else { //STICK_RIGHT
            dStickX = (double) gamepad.right_stick_x;
            if (dStickX == 0) dStickX = .000001; //so don't divide by 0
            dStickY = -(double) gamepad.right_stick_y;
        }

        dAngle = Math.toDegrees(Math.atan(dStickY / dStickX));
        //Stick direction indicated by line
        //  The "0" indicates the angle calculated  above
        //  Must now convert the angle to a polar angle
        //
        //      | /          \ |              |              |
        //      |/ 0        -0\|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |           0 /|              |\ -0
        //      |              |            / |              | \
        //  polar angle=   polar angle=   polar angle=   polar angle=
        //  90-angle       -(angle+90)    -(angle+90)    -angle+90

        /*
        if((dStickX>=0)&&(dStickY>=0)) {
            dPolar=(double)90-dAngle;
        }
        else if((dStickX<0)&&(dStickY>=0)) {
            dPolar=-(dAngle+(double)90);
        }
        else if((dStickX<0)&&(dStickY<0)) {
            dPolar=-(dAngle+(double)90);
        }
        else if((dStickX>=0)&&(dStickY<0)) {
            dPolar=-dAngle+(double)90;
        }
        */
        //Note that the code below can be generalized
        //It is left in this if.then.else format to help reading and future debugging
        if (dStickX >= 0) {
            if (dStickY >= 0) {
                dPolar = (double) 90 - dAngle;
            } else { //(dStickY<0)
                dPolar = -dAngle + (double) 90;
            }
        } else { //dStickX<0
            if (dStickY >= 0) {
                dPolar = -(dAngle + (double) 90);
            } else { //(dStickY<0)
                dPolar = -(dAngle + (double) 90);
            }
        }

        dDirectionStickPolar = dPolar;
        return dPolar;
    }


    private double setRobotDirectionPolar() {
        //From the robot's perspective,it is positioned in at the IMU yaw angle.
        //The direction stick moves the robot in the angle of direction stick.
        //Therefore: must combine these angles so robot moves in direction of direction stick
        //    while maintaining its yaw angle
        //The combination gives the angle, from the robot's perspective, that it must travel.

        //dDirectionStickPolar  goes between 0 to -180 and 0 to 180
        //dRobotYawAngle goes between 0 to -180 and 0 to 180

        double dRobotYawPolar = getNavXYaw();

        double dChk;

        if ((dRobotYawPolar >= 0) && (dRobotYawPolar <= 180)) {
            // robot yaw in positive polar angle
            //    Stick: Direction
            //           ^
            //           |
            //        <- D ->   stick pushed -> +90 deg polar
            //           |
            //           v
            // `  Robot: Yaw
            //
            //              /\ Front
            //             /  \   |
            //            /   /   | robot is facing +45 deg polar from its perspective (yaw)
            //            \  /    |
            //             \/     |
            //        ------------+----------
            //
            //   Together
            //              /\ Front
            //             /  \   |
            //            /   / ->| From the robot's perspective robot must travel 90-45=45 deg polar
            //            \  /    |    this will maintain its yaw, and go in the stick direction
            //             \/     |
            //        ------------+----------
            dRobotDirectionPolar = dDirectionStickPolar - dRobotYawPolar;
        } else { //robot facing -.0~1 to -180
            // robot yaw in negative polar angle
            //    Stick:  Direction
            //           ^
            //           |
            //        <- D ->   stick pushed -> +90 deg polar
            //           |
            //           v
            // `  Robot:  Yaw
            //
            //              /\
            //             /  \   |
            //            /   /   | robot is facing -135 deg polar
            //            \  /    |
            //       Front \/     |
            //        ------------+----------
            //
            //   Together
            //              /\    |
            //             /  \   |
            //            /   / ->| From the robot's perspective robot must travel 90-(-135) deg polar
            //            \  /    |    90-(-135)= 225
            //       Front \/     |    225 is > than 180, so -(360-225) = -135 polar from the robot's perspective
            //        ------------+----------
            //robot in negative polar angle, need to check stick poloar angle
            dChk = dDirectionStickPolar - dRobotYawPolar;
            if (dChk > 180) {
                dRobotDirectionPolar = -(360 - dChk);
            } else {
                dRobotDirectionPolar = dChk;
            }
            //                             logic
            //DirStick   RobotYaw    dChk  tst    Res
            //  90           0        90           90
            //  90          90         0            0
            //  90         180       -90          -90
            //  90        -180       270  >180    -(360-270)=-90
            //  90         -90       180          180
        }
        return dRobotDirectionPolar; //setting this global variable above, and returning it
    }

    private void setMecWheelDirPwr(double dRobotPolarTarget) {
        //This method sets each mecanum wheel's power so that the robot travels in the
        //direction of the passed value (dRobotPolarTarget)
        double dFullPwr = dOmniFullPwr;
        double dScale;


        //correct input
        if (dRobotPolarTarget < -180) {
            //need to go right
            dRobotPolarTarget = (double) 360 + dRobotPolarTarget;
        } else if (dRobotPolarTarget > 180) {
            //need to go left
            dRobotPolarTarget = (double) 360 - dRobotPolarTarget;
        }


        //left front wheel power setting calculation
        //Possible polar target directions:
        //      | /          \ |              |              |
        //      |/            \|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |             /|              |\
        //      |              |            / |              | \
        //LF wheel direction
        //   0=fwd          0=fwd        -90=back        90=fwd
        //  45=fwd        -45=stop      -135=back       135=stop
        //  90=fwd        -90=back      -180=back       180=back

        if ((dRobotPolarTarget >= 0) && (dRobotPolarTarget < 90)) {
            //for this target polar angle range, LF wheel is always fwd
            dDirLFPwr = dFullPwr;
        } else if ((dRobotPolarTarget < 0) && (dRobotPolarTarget >= -90)) {
            //for this target polar angle range, LF wheel goes from fwd to back
            //-90----- -45 ------ 0   polar angle
            //-100------ 0 ------ 100  %full pwr
            dScale = (double) 1 / (double) 45; //because 45 is the zero power
            dDirLFPwr = dFullPwr * ((double) 1 + (dRobotPolarTarget * dScale));
            //polar 0 to -45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 + (0 * (1/45))
            //                    fullpwr * (1 + 0) = fullpwr = fullpwr fwd
            //   when target=-45: fullpwr * (1 + (-45 * (1/45))
            //                    fullpwr * (1 + -1) = 0 stop
            //polar -45 to -90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 + (-90 * (1/45))
            //                    fullpwr * (1 + -2) = fullpwr * -1 = -fullpwr = fullpwr back
        } else if ((dRobotPolarTarget < -90) && (dRobotPolarTarget >= -180)) {
            //for this target polar angle range, LF wheel is always back
            dDirLFPwr = -dFullPwr;
        } else if ((dRobotPolarTarget >= 90) && (dRobotPolarTarget <= 180)) {
            //for this target polar angle range, LF wheel goes from fwd to back
            //90----- 135 ----- 180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirLFPwr = dFullPwr * ((double) 3 - (dRobotPolarTarget * dScale));
            //polar 90 to 135, wheel %pwr will go from 100 to 0
            //   when target=90:  fullpwr * (3 - (90 * (1/45))
            //                    fullpwr * (3 - (2)) = fullpwr * 1 = fullpwr fwd
            //   when target=135: fullpwr * (3 - (135* (1/45))
            //                    fullpwr * (3 - (3)) = 0 stop
            //polar 90 to 180, above will go from 0 to -1 because
            //   when target=180: fullpwr * (3 - (180 * (1/45))
            //                    fullpwr * (3 - (4)) = fullpwr * -1 = fullpwr back
        }


        //right front
        //Possible polar target directions:
        //      | /          \ |              |              |
        //      |/            \|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |             /|              |\
        //      |              |            / |              | \
        //RF wheel direction
        //   0=fwd          0=fwd        -90=fwd         90=back
        //  45=stop       -45=fwd       -135=stop       135=back
        //  90=back       -90=fwd       -180=back       180=back

        if ((dRobotPolarTarget >= 0) && (dRobotPolarTarget < 90)) {
            //for this target polar angle range, RF wheel goes from fwd to back
            //0---------45--------90   polar angle
            //100------ 0------- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirRFPwr = dFullPwr * ((double) 1 - (dRobotPolarTarget * dScale));
            //polar 0 to 45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 - (0 * (1/45))
            //                    fullpwr * (1-0) = fullpwr = fullpwr fwd
            //   when target=45:  fullpwr * (1 - (45 * (1/45))
            //                    fullpwr * (1-1) = 0 stop
            //polar 45 to 90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 - (90 * (1/45))
            //                    fullpwr * (1-2) = fullpwr * -1 = -fullpwr = fullpwr back
        } else if ((dRobotPolarTarget < 0) && (dRobotPolarTarget >= -90)) {
            //for this target polar angle range, RF wheel is always fwd
            dDirRFPwr = dFullPwr;
        } else if ((dRobotPolarTarget < -90) && (dRobotPolarTarget >= -180)) {
            //for this target polar angle range, RF wheel goes from fwd to back
            //-90---- -135 ---- -180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirRFPwr = dFullPwr * ((double) 3 + (dRobotPolarTarget * dScale));
            //polar -90 to -135, wheel %pwr will go from 100 to 0
            //   when target=-90: fullpwr * (3 + (-90 * (1/45))
            //                    fullpwr * (3 + (-2)) = fullpwr * 1 = fullpwr fwd
            //   when target=-135:fullpwr * (3 + (-135* (1/45))
            //                    fullpwr * (3 + (-3)) = 0 stop
            //polar -90 to -180, above will go from 0 to -1 because
            //   when target=-180:fullpwr * (3 + (-180 * (1/45))
            //                    fullpwr * (3 + (-4)) = fullpwr * -1 = fullpwr back
        } else if ((dRobotPolarTarget >= 90) && (dRobotPolarTarget <= 180)) {
            //for this target polar angle range, RF wheel is always back
            dDirRFPwr = -dFullPwr;
        }

        // left back
        //Possible polar target directions:
        //      | /          \ |              |              |
        //      |/            \|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |             /|              |\
        //      |              |            / |              | \
        //LB wheel direction
        //   0=fwd          0=fwd        -90=fwd         90=back
        //  45=stop       -45=fwd       -135=stop       135=back
        //  90=back       -90=fwd       -180=back       180=back

        if ((dRobotPolarTarget >= 0) && (dRobotPolarTarget < 90)) {
            //for this target polar angle range, LB wheel goes from fwd to back
            //0---------45--------90   polar angle
            //100------ 0------- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirLBPwr = dFullPwr * ((double) 1 - (dRobotPolarTarget * dScale));
            //polar 0 to 45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 - (0 * (1/45))
            //                    fullpwr * (1-0) = fullpwr = fullpwr fwd
            //   when target=45:  fullpwr * (1 - (45 * (1/45))
            //                    fullpwr * (1-1) = 0 stop
            //polar 45 to 90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 - (90 * (1/45))
            //                    fullpwr * (1-2) = fullpwr * -1 = -fullpwr = fullpwr back
        } else if ((dRobotPolarTarget < 0) && (dRobotPolarTarget >= -90)) {
            //for this target polar angle range, LB wheel is always fwd
            dDirLBPwr = dFullPwr;
        } else if ((dRobotPolarTarget < -90) && (dRobotPolarTarget >= -180)) {
            //for this target polar angle range, LB wheel goes from fwd to back
            //-90---- -135 ---- -180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirLBPwr = dFullPwr * ((double) 3 + (dRobotPolarTarget * dScale));
            //polar -90 to -135, wheel %pwr will go from 100 to 0
            //   when target=-90: fullpwr * (3 + (-90 * (1/45))
            //                    fullpwr * (3 + (-2)) = fullpwr * 1 = fullpwr fwd
            //   when target=-135:fullpwr * (3 + (-135* (1/45))
            //                    fullpwr * (3 + (-3)) = 0 stop
            //polar -90 to -180, above will go from 0 to -1 because
            //   when target=-180:fullpwr * (3 + (-180 * (1/45))
            //                    fullpwr * (3 + (-4)) = fullpwr * -1 = fullpwr back
        } else if ((dRobotPolarTarget >= 90) && (dRobotPolarTarget <= 180)) {
            //for this target polar angle range, LB wheel is always back
            dDirLBPwr = -dFullPwr;
        }


        //right back
        //      | /          \ |              |              |
        //      |/            \|              |              |
        //  ----+----      ----+----      ----+----      ----+----
        //      |              |             /|              |\
        //      |              |            / |              | \
        //   0=fwd          0=fwd        -90=back        90=fwd
        //  45=fwd        -45=stop      -135=back       135=stop
        //  90=fwd        -90=back      -180=back       180=back

        if ((dRobotPolarTarget >= 0) && (dRobotPolarTarget < 90)) {
            //for this target polar angle range, RB wheel is always fwd
            dDirRBPwr = dFullPwr;
        } else if ((dRobotPolarTarget < 0) && (dRobotPolarTarget >= -90)) {
            //for this target polar angle range, RB wheel goes from fwd to back
            //-90----- -45 ------ 0   polar angle
            //-100------ 0 ------ 100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirRBPwr = dFullPwr * ((double) 1 + (dRobotPolarTarget * dScale));
            //polar 0 to -45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 + (0 * (1/45))
            //                    fullpwr * (1 + 0) = fullpwr = fullpwr fwd
            //   when target=-45: fullpwr * (1 + (-45 * (1/45))
            //                    fullpwr * (1 + -1) = 0 stop
            //polar -45 to -90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 + (-90 * (1/45))
            //                    fullpwr * (1 + -2) = fullpwr * -1 = -fullpwr = fullpwr back
        } else if ((dRobotPolarTarget < -90) && (dRobotPolarTarget >= -180)) {
            //for this target polar angle range, RB wheel is always back
            dDirRBPwr = -dFullPwr;
        } else if ((dRobotPolarTarget >= 90) && (dRobotPolarTarget <= 180)) {
            //for this target polar angle range, RB wheel goes from fwd to back
            //90----- 135 ----- 180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale = (double) 1 / (double) 45;
            dDirRBPwr = dFullPwr * ((double) 3 - (dRobotPolarTarget * dScale));
            //polar 90 to 135, wheel %pwr will go from 100 to 0
            //   when target=90:  fullpwr * (3 - (90 * (1/45))
            //                    fullpwr * (3 - (2)) = fullpwr * 1 = fullpwr fwd
            //   when target=135: fullpwr * (3 - (135* (1/45))
            //                    fullpwr * (3 - (3)) = 0 stop
            //polar 90 to 180, above will go from 0 to -1 because
            //   when target=180: fullpwr * (3 - (180 * (1/45))
            //                    fullpwr * (3 - (4)) = fullpwr * -1 = fullpwr back
        }
    }


    private void setOmniYawPwr(Gamepad gamepad, boolean bYawStick) {
        //Yaw stick rotates the robot more clock wise or counter clock wise from its current position
        //1.  When the yaw stick is pressed (not in dead zone) rotate robot
        //2.  When the stick is in the dead zone (not pressed), use the NavX Yaw PID controller
        //    to hold the position set when the yaw stick is first released

        double dStickX;

        //check if yaw stick is released
        if (chkStickDeadZone(gamepad, bYawStick)) {
            //we are in dead zone
            //check if yaw stick was pushed on last check
            if (bYawPressed) {
                //stick in dead zone, stick was pressed on last check, so yaw stick is now released
                //set the NavX yaw PID controller to try to hold this yaw position
                double dMomentumCompensation = 0;
                if (nRobotSpin == NOSPIN) dMomentumCompensation = 0;
                else if (nRobotSpin == CW) dMomentumCompensation = dMomentumAdjustment;
                else if (nRobotSpin == CCW) dMomentumCompensation = -dMomentumAdjustment;

                //SkyStone v5.2 incompatibility
                /*enableNavXYawPID(true);*/
                setpointNavXYaw(getNavXYaw() + dMomentumCompensation);
                dYawPwr = 0;
            } else { //gbYawPressed is false so trying to maintain yaw position
                //stick in dead zone, last time was not pressed, so maintain position
                dYawPwr = getNavXCorrectionPwr();
            }
            //we are in dead zone on this check
            bYawPressed = false;
        } else {
            //not in dead zone
            //yaw stick IS now pressed:
            //   1.  yaw pid hold setting needs to change. so disable yaw pid controller;
            //   2.  need to spin robot without yaw pid controller, will enable again when yaw stick released
            //SkyStone v5.2 incompatibility
            //enableNavXYawPID(true);
            ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
            //     -          -
            //  -  L  +    -  R
            //     +          +
            if (bYawStick == STICK_LEFT)
                dStickX = (double) gamepad.left_stick_x;
            else
                dStickX = (double) gamepad.right_stick_x;

            //when dStickX is negative, need to rotate counter clock wise
            if (dStickX < 0) {
                dYawPwr = -(dOmniFullPwr * YAW_DIR_SCALAR);
                nRobotSpin = CCW;
            } else {
                dYawPwr = dOmniFullPwr * YAW_DIR_SCALAR;
                nRobotSpin = CW;
            }
            bYawPressed = true; //set so know what happened when this was run
        }

    }


    boolean chkStickDeadZone(Gamepad gamepadToChk, boolean bControlStickToChk) {

        //check if sticks are in dead zone
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
        if (bControlStickToChk == STICK_LEFT) {
            if (((gamepadToChk.left_stick_x <= OMNI_STICK_DEADZONE) && (gamepadToChk.left_stick_x >= -OMNI_STICK_DEADZONE)) &&
                    ((-gamepadToChk.left_stick_y <= OMNI_STICK_DEADZONE) && (-gamepadToChk.left_stick_y >= -OMNI_STICK_DEADZONE)))
                return true;
        } else {
            if (((gamepadToChk.right_stick_x <= OMNI_STICK_DEADZONE) && (gamepadToChk.right_stick_x >= -OMNI_STICK_DEADZONE)) &&
                    ((-gamepadToChk.right_stick_y <= OMNI_STICK_DEADZONE) && (-gamepadToChk.right_stick_y >= -OMNI_STICK_DEADZONE)))
                return true;
        }
        //out side of dead zone
        return false;
    }

    //SkyStone v5.2 incompatibility
    /*
    public void chkResetRobotYawZero(Gamepad gamepadToChk) {
        long lCurrentTime=System.currentTimeMillis();

        //see if okay to reset yaw
        if((lCurrentTime-lBackTimestamp)<200)  return;
        if(!bHasMecanum) return;  //only for macanum sheels

        if (gamepadToChk.back) { //back button hit
            if(nNavXStatus==NAVX_AS_YAWPID) {
                //if got here, can reset yaw
                setMecWheelOff(mtrLeftBack, mtrLeftFront, mtrRightBack, mtrRightFront);
                //wait for robot to stop moving..
                navx_device.zeroYaw();
                //while((System.currentTimeMillis()-lCurrentTime)<300) wait();
                dYawAngleToHold = 0;
                yawPIDController.setSetpoint(dYawAngleToHold);
                bYawPressed = false;
                dYawPwr = 0;
                dDirLFPwr = dDirLBPwr = dDirRFPwr = dDirRBPwr = 0;
            } else { //not using DIM.  use to control loosing gyro connection

            }

            lBackTimestamp=lCurrentTime;
        }
    }
    */

    private void setMecWheelOff(
            DcMotor mtrLeftBack, DcMotor mtrLeftFront, DcMotor mtrRightBack, DcMotor mtrRightFront) {
        mtrLeftFront.setPower(0);
        mtrLeftBack.setPower(0);
        mtrRightFront.setPower(0);
        mtrRightBack.setPower(0);

    }

    private void setMecWheelDirectionPwrZero() {
        dDirLFPwr = 0;
        dDirRFPwr = 0;
        dDirLBPwr = 0;
        dDirRBPwr = 0;
    }


    public void initDrive(int nInitialControlMode, int nInitialSpeed, int nInitOrientation) {

        //if invalid parameters set, the following will be used
        nControlMode = TANK_CONTROL;
        dTankSpeedDir = TANK_FASTFWD;

        //initialize control, spped, orientation
        if (nInitialControlMode == TANK_CONTROL) {
            nControlMode = TANK_CONTROL;
            if ((nInitialSpeed == START_FAST) && (nInitOrientation == START_FWD)) {
                dTankSpeedDir = TANK_FASTFWD;
            } else if ((nInitialSpeed == START_FAST) && (nInitOrientation == START_REV)) {
                dTankSpeedDir = TANK_FASTREV;
            } else if ((nInitialSpeed == START_SLOW) && (nInitOrientation == START_FWD)) {
                dTankSpeedDir = TANK_SLOWFWD;
            } else if ((nInitialSpeed == START_SLOW) && (nInitOrientation == START_REV)) {
                dTankSpeedDir = TANK_SLOWREV;
            }
        } else if (nInitialControlMode == OMNI_CONTROL) {
            nControlMode = OMNI_CONTROL;
            if (nInitialSpeed == START_FAST) {
                dOmniFullPwr = OMNI_FAST_PWR;
            } else if (nInitialSpeed == START_SLOW) {
                dOmniFullPwr = OMNI_SLOW_PWR;
            }
            if (nInitOrientation == START_RIGHT_HAND) {
                bDirectionStick = STICK_RIGHT;
            } else if (nInitOrientation == START_LEFT_HAND) {
                bDirectionStick = STICK_LEFT;
            }
        }
        return;
    }


    public double getYawAngleToHold() {
        return dYawAngleToHold;
    }

    public double getYawPwr() {
        return dYawPwr;
    }

    public int getControlMode() {
        return nControlMode;
    }

    public double getDirectionStickPolarAngle() {
        return dDirectionStickPolar;
    }

    public double getResultantLFPwr() {
        return (dResultantLFPwr);
    }

    public double getResultantLBPwr() {
        return (dResultantLBPwr);
    }

    public double getResultantRFPwr() {
        return (dResultantRFPwr);
    }

    public double getResultantRBPwr() {
        return (dResultantRBPwr);
    }

    public String getTankMovement() {
        if (nTankMovement == TANK_NO_MOVEMENT) return ("No Tank Movement");
        if (nTankMovement == TANK_MOVEMENT) return ("Tank Movement");
        if (nTankMovement == TANK_LATERAL_MOVEMENT) return ("Tank Lateral Movement");
        return "---------";
    }

    public String getNavXStatus() {


        //SkyStone v5.2 incompatibility
        /*
        if(nNavXStatus==NAVX_AS_YAWPID) return ("NavX with yawPIDController");
        else if(nNavXStatus==NAVX_AS_GYRO) return ("NavX acting as Gyro");
        else return ("No directional control");
        */
        if (nNavXStatus == NAVX_AS_GYRO) return ("NavX acting as Gyro");
        else return ("No directional control");

    }


    public boolean isNavXConnected() {
        //SkyStone v5.2 incompatibility
        /*
        if(nNavXStatus==NAVX_AS_YAWPID) {
            return (navx_device.isConnected());
        } else {
            //no equivalent without dim, so send true an pray
            return true;
        }
        *
         */
        return true;
    }

    public double getNavXYaw() {
        //SkyStone v5.2 incompatibility
        /*if(nNavXStatus==NAVX_AS_YAWPID) {
            if(navx_device.isConnected())
                return (double) (navx_device.getYaw());
            else return 0;
        } else */
        if (nNavXStatus == NAVX_AS_GYRO) {
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return (double) (-(angles.firstAngle)); //Darn FIRST implementation returns a neg angle
        } else {
            return (double) 0;
        }
    }

    public void closeNavX() {
        //SkyStone v5.2 incompatibility
        /*
        if(nNavXStatus==NAVX_AS_YAWPID) {
            if(yawPIDController.isEnabled()) yawPIDController.enable(false);
            yawPIDController.close();
            navx_device.close();
        }
        else {
            //nothing equivalent in gyro classes

        }
         */

    }

    //SkyStone v5.2 incompatibility
    /*
    public void enableNavXYawPID(boolean bEnable)  {
        if(nNavXStatus==NAVX_AS_YAWPID) {
            if(bEnable==true) {
                if(!yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
        }
        //no equivalent without DIM
    }
    */
    public void setpointNavXYaw(double dAngleToSet) {
        //SkyStone v5.2 incompatibility
        /*
        if(nNavXStatus==NAVX_AS_YAWPID) {
           yawPIDController.setSetpoint(dAngleToSet);
        }
        */
        //must set dYawAngleToHold regardless
        dYawAngleToHold = dAngleToSet;
    }

    public double getNavXCorrectionPwr() {
        //prior to calling this, the user MUST call setpointNavXYaw to set the target angle
        //   the difference between the robot's current angle and the target angle
        //   determines the magnitude and sign of the output returned.
        //SkyStone v5.2 incompatibility
        /*
        if(nNavXStatus==NAVX_AS_YAWPID) {
            if (navx_device.isConnected()) {
                navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
                try {
                    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                        if (yawPIDResult.isOnTarget()) {
                            //telemetry.addData("PIDOutput", df.format(dPwr) + ", " +
                            // df.format(dPwr));
                            return (double) 0;
                        } else {
                            //bigger output means increase increase power to left so chassis goes right
                            double output = yawPIDResult.getOutput();
                            return (output);
                        }
                    } else {
                        //A timeout occurred
                        Log.w("navXMecDrive", "Yaw PID waitForNewUpdate() TIMEOUT.");
                        return (double) 0;
                    }
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                    return (double) 0;

                } finally {

                }
            } else { //navx not connected
                return 0;
            }
        } else */
        if (nNavXStatus == NAVX_AS_GYRO) {
            return calcCrudeCorrectionPwr(dYawAngleToHold, getNavXYaw());
        } else {
            return 0;
        }
    }

    private double calcCrudeCorrectionPwr(double dTargetPolar, double dCurrentPolar) {
        //crude corrective power calculation.... change or redo as you see fit
        //
        // the returned value is ment to add to the left wheels ( and subtract from the right wheels
        //
        //
        //   1.0  |
        // p  .9  |
        // w  .8  |
        // r  .7  |------------------------- CCCAP
        //    .6  |       * * * * * *  *  *
        //    .5  |      *
        //    .4  |     *
        //    .3  |    *
        //    .2  |  *
        //    .1  | *
        //    0   +--+--+--+--+--+--+--+--+--+
        //        0 10 20 30 40 50 60 70 80 90
        //             polar deg difference

        // y = d(x * x)
        //
        //
        // pwr| diff| CCSCALAR
        // y  |  X  |  d
        //----+-----+--------
        //  0    0
        //       2
        // .088  5    .0035
        // .17   7    .0035
        // .35  10    .0035
        // .5   12    .0035
        // .79  15    .0035
        // .87  17    .0035
        //1.20  20    .0035

        double dY = 0, dX = 0;
        int nDir = 0;

        dCurrentPolar = fixPolar(dCurrentPolar);
        dTargetPolar = fixPolar(dTargetPolar);
        //first find diff
        if ((dCurrentPolar >= 0) && (dCurrentPolar <= 180)) {
            //At a  positive current  polar angle
            if (dTargetPolar <= dCurrentPolar) {
                //if here need to move counter clockwise
                dX = dTargetPolar - dCurrentPolar; //make dX neg
                nDir = -1; //Counter clockwise movement
            } else {  //dTargetPolar>dCurrentPolar
                //if here need to move clockwise
                dX = dCurrentPolar - dTargetPolar; //make dX pos
                nDir = 1; //Clockwise movement
            }
        } else if ((dCurrentPolar < 0) && (dCurrentPolar >= -180)) { //can combine with above conditional
            //do this way to help debugging
            //At a negative current  polar angle
            if (dTargetPolar <= dCurrentPolar) {
                //if here need to move counter clockwise
                dX = dTargetPolar - dCurrentPolar; //make dX pos
                nDir = -1;
            } else { //dTargetPolar>dCurrentPolar)
                //if here need to move clockwise
                dX = dCurrentPolar - dTargetPolar; //make dX pos
                nDir = 1; //Clockwise movement
            }
        }
        dY = (nDir) * (dX * dX) * CCSCALAR;
        if (dY > CCCAP) return CCCAP;
        else return dY;
    }

    public double fixPolar(double dPolarToFix) {

        double dRes = dPolarToFix;

        dPolarToFix %= 360.0; //normalize  to 0 to 360;

        if (dPolarToFix > (double) 180) {
            //more than 180, so polar is actually negative
            dRes = dPolarToFix - (double) 360;
        } else if (dPolarToFix < -(double) 180) {
            dRes = dPolarToFix + (double) 360;
        }
        return dRes;
    }
/*
    //SkyStone v5.2 incompatibility
    public boolean calibrateNavXYawPID(OpMode opMode) {
        try {
            //navx_device = AHRS.getInstance(opMode.hardwareMap.deviceInterfaceModule.get("dim"),
            //        NAVX_DIM_I2C_PORT,
            //        AHRS.DeviceDataType.kProcessedData,
            //        NAVX_DEVICE_UPDATE_RATE_HZ);

            navx_device = AHRS.getInstance(opMode.hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData);
            opMode.telemetry.addData("navX-Micro", "Startup Calibration");
            opMode.telemetry.update();
            while (!calibration_complete) {
                // navX-Micro Calibration completes automatically ~15 seconds after it is
                //powered on, as long as the device is still.  To handle the case where the
                //navX-Micro has not been able to calibrate successfully, hold off using
                //the navX-Micro Yaw value until calibration is complete.

                calibration_complete = !navx_device.isCalibrating();
                if (!calibration_complete) {
                    opMode.telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                }
                opMode.telemetry.update();
                //opMode.wait(50);
                //opMode.idle();
            }
            navx_device.zeroYaw();

            opMode.telemetry.addData("navX-Micro", "Startup Calibration Complete");
            opMode.telemetry.update();

            //Create a PID Controller which uses the Yaw Angle as input.
            yawPIDController = new navXPIDController(navx_device,
                    navXPIDController.navXTimestampedDataSource.YAW);

            yawPIDController.setSetpoint(dYawAngleToHold);
            yawPIDController.setContinuous(true);
            yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
            yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
            yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
            yawPIDController.enable(true);
            return true;
        }
        catch (IllegalArgumentException exDIM) {
            opMode.telemetry.addData("NO DIM!! navX-Micro", "%s", exDIM.toString());
            opMode.telemetry.update();
            //No Device Interface Module named dim found!
            return false;
        }
    }
*/


    public boolean calibrateNavXAsGyro(OpMode opMode) {
        try {
            long lTimeStart = System.currentTimeMillis();
            boolean bTimeout = false;

            navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope) navxMicro;

            // The gyro automatically starts calibrating. This takes a few seconds.
            opMode.telemetry.addData("Gyro:", "Calibrating. Do Not Move!");
            opMode.telemetry.update();

            // Wait until the gyro calibration is complete
            while (navxMicro.isCalibrating() && !bTimeout) {
                opMode.telemetry.addData("calibrating", "...");
                opMode.telemetry.update();
                if ((System.currentTimeMillis() - lTimeStart) < 10000)
                    bTimeout = true; //10 second timeout
                Thread.sleep(50);
            }
            if (bTimeout) return false;
            return true;
        } catch (IllegalArgumentException exNavX) {
            opMode.telemetry.addData("argument", "%s", exNavX.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return false;
        } catch (InterruptedException exInt) {
            opMode.telemetry.addData("interrupt", "%s", exInt.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return false;
        }

    }

    void resetMotorsTeleop(boolean bUseEncoders) {

        /*4 motors, so must sync so use encoders, do NOT stop and reset or will not run*/
        if (bUseEncoders) setMotorsWithEncoders(false);
        else setMotorsWithoutEncoders();
        if (bHasMecanum) {
            mtrRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        mtrLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void resetMotorsAuton() {
        if (bHasMecanum) {
            mtrLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        mtrLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorsWithEncoders(true);
    }


    void setMotorsWithEncoders(boolean bReset) {
        if (bHasMecanum) {
            mtrLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (bReset) mtrLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mtrRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (bReset) mtrRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        mtrRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (bReset) mtrRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        mtrLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (bReset) mtrLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void setMotorsWithoutEncoders() {
        if (bHasMecanum) {
            mtrLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        mtrLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
    public void moveRobot(LinearOpMode opMode, String msg, double speed, int distance, int desired_angle, int desired_yaw,
                          int timeout) {
        long lMarkMilliS;
        String msg2;
        double heading;
        double myspeed;
        double factor;

        //Left Back Motor
        mtrLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Left Forward Motor
        mtrLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Back Motor
        mtrRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Right Front Motor
        mtrRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lMarkMilliS=System.currentTimeMillis() + timeout;
        int pos = Math.abs(mtrLeftBack.getCurrentPosition());
        heading = getNavXYaw();

        if(desired_yaw == 180) {
            while ((System.currentTimeMillis() < lMarkMilliS) && heading < 176 && opMode.opModeIsActive()
            ) {
                heading = getNavXYaw();
                if (desired_yaw == 180) {
                    msg2 = "turn around";
                    mtrLeftBack.setPower(speed);
                    mtrLeftFront.setPower(speed);
                    mtrRightBack.setPower(-speed);
                    mtrRightFront.setPower(-speed);
                }
            }
        }


        // ----

            while((System.currentTimeMillis() < lMarkMilliS) &&
                pos < distance &&  opMode.opModeIsActive()
        ) {
            pos = Math.abs(mtrLeftBack.getCurrentPosition());
            if (pos < 190) {
                factor = Math.sqrt((pos + 10.0) / 200.0);
            } else if (pos > distance - 190) {
                factor = (distance - pos + 10)/200.0;
            } else {
                factor = 1.0;
            }

            heading = getNavXYaw();
            myspeed = speed * factor;
            if (myspeed < 0.1)
                myspeed = 0.1;
            if (desired_yaw == 90) {
                msg2 = "turn right";
                mtrLeftBack.setPower(myspeed);
                mtrLeftFront.setPower(myspeed);
                mtrRightBack.setPower(-myspeed);
                mtrRightFront.setPower(-myspeed);
            } else if (desired_yaw == -90) {
                msg2 = "turn left";
                mtrLeftBack.setPower(-myspeed);
                mtrLeftFront.setPower(-myspeed);
                mtrRightBack.setPower(myspeed);
                mtrRightFront.setPower(myspeed);
            }
            else if (desired_angle == 0) {
                msg2 = "move forward";
                //Go forward
                mtrLeftBack.setPower(myspeed*(1-heading/100));
                mtrLeftFront.setPower(myspeed*(1-heading/100));
                mtrRightBack.setPower(myspeed*(1+heading/100));
                mtrRightFront.setPower(myspeed*1+heading/100);
            } else if (desired_angle == 180) {
                msg2 = "move backwards";
                //go backwards
                mtrLeftBack.setPower(-myspeed*(1+heading/100));
                mtrLeftFront.setPower(-myspeed*(1+heading/100));
                mtrRightBack.setPower(-myspeed*(1-heading/100));
                mtrRightFront.setPower(-myspeed*(1-heading/100));
            } else if (desired_angle == 90) {
                msg2 = "move right";
                mtrLeftBack.setPower(-myspeed*(1+heading/100));
                mtrLeftFront.setPower(myspeed*(1-heading/100));
                mtrRightBack.setPower(myspeed*(1+heading/100));
                mtrRightFront.setPower(-myspeed*(1-heading/100));
            } else if (desired_angle == -90) {
                msg2 = "move left";
                mtrLeftBack.setPower(myspeed*(1-heading/100));
                mtrLeftFront.setPower(-myspeed*(1+heading/100));
                mtrRightBack.setPower(-myspeed*(1-heading/100));
                mtrRightFront.setPower(myspeed*(1+heading/100));
            }
            else {
                msg2 = "no movement";
            }
            opMode.telemetry.addData(msg2 + " "+ msg + " M pos:", "%.2f, %d, %d, %d, %d, %.2f", heading,
                    mtrLeftBack.getCurrentPosition(), mtrRightBack.getCurrentPosition(),
                    mtrRightFront.getCurrentPosition(), mtrLeftFront.getCurrentPosition(), last_hue);
            opMode.telemetry.update();

        }
        mtrLeftBack.setPower(0);
        mtrRightBack.setPower(0);
        mtrLeftFront.setPower(0);
        mtrRightFront.setPower(0);

    }



    //elevator
    public void moveElev(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
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
    public void moveIntake(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
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

    //conveyor
    public void moveConveyor(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
        long lMarkMilliS;
        lMarkMilliS=System.currentTimeMillis();
        //Launcher Motor (encoder)
        mtrConveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
    public void moveLauncher(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
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


    public boolean detectSkystone(OpMode opMode) {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        last_hue = hsvValues[0];

        opMode.telemetry.addData("Hue Value", hsvValues[0]);
        opMode.telemetry.update();

        if (hsvValues[0] < 43)
            return false;
        else
            return true;
    }




    public void moveSrvo(LinearOpMode opMode, String msg, double power, int timeout) {
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
    */

}