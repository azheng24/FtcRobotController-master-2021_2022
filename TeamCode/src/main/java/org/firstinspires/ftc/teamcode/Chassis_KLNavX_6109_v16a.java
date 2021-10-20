
package org.firstinspires.ftc.teamcode;

//From the Kauai Labs Docs
// https://pdocs.kauailabs.com/navx-micro/software/libraries/android-library-2016-version/
//Add the following to  teamcode/build.release.gradle
//  Add the following to the repositories section
//   repositories {
//       flatDir {
//           dirs 'libs', 'C:\\Users\\Robot\\navx-micro\\android\\libs'
//       }
//   }
// Add the following to to dependencies section
//   dependencies {
//         compile (name:'navx_ftc-release', ext:'aar')
//   }
//
//after entering the above, the followins lines should compile

 import android.graphics.Color;
 import android.util.Log;


 import com.kauailabs.navx.ftc.AHRS;
 import com.kauailabs.navx.ftc.AHRS;
 import com.kauailabs.navx.ftc.navXPIDController;
 import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Gamepad;
 import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
 import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
 import com.qualcomm.robotcore.hardware.NormalizedRGBA;
 import com.qualcomm.robotcore.hardware.SwitchableLight;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

 import static com.kauailabs.navx.ftc.AHRS.DeviceDataType;
 import static com.kauailabs.navx.ftc.AHRS.getInstance;



//the next line is from the FIRST library for the navX.  This code uses both


 /**
 * A field-centric omni drive AND a robot-centric tank drive robot chassis
 * Mecanum wheels, 4 motors w/encoders, and a Kauai Labs navX required
 * @version 9
 * @author  FTC Teams 6109 and 6963
 *
 */
public class Chassis_KLNavX_6109_v16a {
    //For those using a Modern Robotics Device Interface Module
    // This is the port on the Core Device Interface Module        */
    // in which the navX-Model Device is connected.  Modify this  */
    //private final int NAVX_DIM_I2C_PORT = 0;
    //NOTE:  for the 2020-2021 season, Modern Robotics modules are no longer supported

    private AHRS navX;  //Kauai Labs library
    private navXPIDController yawPIDController;
    //private ElapsedTime runtime = new ElapsedTime();

    //Kauai Labs navX constants
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 0.0d;
    private final double TOLERANCE_DEGREES = 1.0d; //when within this tolerance, no more heading
                                                 //corrections are made
     private final int NAVX_TIMEOUT=200;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0d;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0d;

    //To tune below
    //create auton program to rotate 90 deg
    //turn I off Yawpido=0
    //adust ypidp to get to close to angle without passing it
    //adust ypidi to get the rest of the way to 90
     //0.3d is previous
    private final double YAW_PID_P = 0.002;
    //previous: 0.0000d
    private final double YAW_PID_I = 0.00001d;
    private final double YAW_PID_D = 0.0d;

    //FIRST library
    IntegratingGyroscope gyro;

    //Motors
    private DcMotor mtrLeftBack = null; //LB
    private DcMotor mtrRightBack = null; //RB
    private DcMotor mtrLeftFront = null; //LF
    private DcMotor mtrRightFront = null; //RF

     //package-private constants... used for this method
    final int CHASSIS_LEFT_FWD = 1; //Left wheels move forward when apply positive voltage
                                           //Includes NeverRest 20 orbitals
    final int CHASSIS_RIGHT_FWD= 2; //Right wheels move forward when apply positive voltage
                                           //Includes NeverRest 3.7 orbitals
    final int OMNI_CONTROL = 1;
    final int TANK_CONTROL = 2;
    final int MTR_LF=1, MTR_RF=2, MTR_LB=3, MTR_RB=4;


     //Tank drive constants
    private final int TANK_OPTION_FWD = 100;
    private final int TANK_OPTION_REV = -100;
    private final double TANK_FAST_PWR = 0.35d;  //fastest pwr for tank drive
    private final double TANK_SLOW_PWR = 0.24d;  //slowest pwr for tank drive
    private final double TANK_CRAWL_PWR = TANK_SLOW_PWR; //pwr to use for crawling
    private final int TANK_NO_MOVEMENT = 0,TANK_VERTICAL_MOVEMENT = 1,  TANK_HORIZONTAL_MOVEMENT = 2,
                       TANK_SPIN_MOVEMENT =3, TANK_TURN_MOVEMENT=4;
    private final float TANK_STICK_DEADZONE = (float) .67;
    //Tank drive global variables
    private double gdTankPwrMagnitude = TANK_FAST_PWR; //magnitude of tank power,no direction
    private int gnTankOption = TANK_OPTION_FWD; //direction of tank
    private int gnTankMovement = TANK_NO_MOVEMENT;
    private int gnTankPrevMove = TANK_NO_MOVEMENT;

    //Omni drive constants
    private final double OMNI_FAST_PWR = 0.3; //fastest pwr for omni drive
    private final double OMNI_SLOW_PWR = 0.5;  //slowest pwr for omni drive
    private final double OMNI_CRAWL_PWR = OMNI_SLOW_PWR; //pwr to use for crawling
    private final float OMNI_STICK_DEADZONE = (float) .33;
    private final boolean STICK_RIGHT = false, STICK_LEFT = true;
    private final double STICK_IN_DEADZONE = 99999.99; //not allowed polar angle
    private final int CCW = -1, CW = 1, NOSPIN = 0;
    private final int DIR_LAST_NONE=0, DIR_LAST_STICK=1, DIR_LAST_BTN=2;
    private final int BTN_RIGHT=0, BTN_LEFT=1, BTN_UP=2, BTN_DOWN=3, BTN_NONE=4;
    //Omni drive global variables
    private boolean gbRotationStkOrBtnPressed = false; //previous read of yaw stick
    private double gdOmniPwrMagnitude = OMNI_FAST_PWR;
    private int gnDirInputLast=DIR_LAST_NONE;
    private double gdLFPwrAdj=1d, gdLBPwrAdj=1d, gdRFPwrAdj=1d, gdRBPwrAdj=1d;
    private int gnLastBtnPressed=BTN_NONE;

    //gamepad global variables
    private double gdDirectionStickPolar;
    private boolean gbOrientationStick = STICK_LEFT, gbDirectionStick = STICK_RIGHT;
    private boolean gbDontUseLeftTrigBmpr = false;

    //Common constants
    private final double ROTATION_PWR_RATIO = (double) .67;
    //private final double ROTATION_PWR_RATIO = (double) .85;
    private final double PWR_LEVELS=3d;  //3: MINLVL--intvl--LVL--intvl--LVL-intvl---MAXLVL
    private final double TANK_PWR_INTVL = (TANK_FAST_PWR-TANK_SLOW_PWR)/PWR_LEVELS;
    private final double OMNI_PWR_INTVL = (OMNI_FAST_PWR-OMNI_SLOW_PWR)/PWR_LEVELS;
    final int NAVX_NONE = 0,  NAVX_KAUAILABS_LIB = 1, NAVX_QC_LIB = 2, NAVX_ERROR=9;
    private final long CONTROL_SENSITIVITY = 200; //miliseconds to wait before reading next control

    //Common global variables
    private int gnChassisLayout;
    private double gdRobotDirectionPolar = 0;
    private int gnRobotSpin;
    private double gdMomentumAdjustmentDeg = 0, gdTankMomentumAdjustmentDeg=0;
    private int gnNavXStatus = NAVX_NONE;
    private double gdYawAngleToHold = 0;
    private double gdRotationPwr; //power for each wheel to get desired yaw to hold
    private double //power for each wheel from direction controls
            gdDirectionLFPwr = 0, gdDirectionRFPwr = 0,
            gdDirectionLBPwr = 0, gdDirectionRBPwr = 0;
    private double //power for each wheel adding yaw and direction, then scaling the largest to 1
            gdResultantLFPwr = 0, gdResultantRFPwr = 0,
            gdResultantLBPwr = 0, gdResultantRBPwr = 0;
    private int gnControlMode = OMNI_CONTROL;
    private double gdRobotWeightLbs;

    //global timestamps
    //private long lBackTimestamp = System.currentTimeMillis(); //button timestamp
    private long glLastControlModePress = System.currentTimeMillis(); //control timestamp
    //private long glAutonTimestamp = System.currentTimeMillis();

     //below only for crudeYawCorrection
    private long glCrudeYawTimeLast =0;
    private double gdCrudePPwr =0d, gdCrudeIPwr=0d;
    private double gdCrudeYawDistLast =0d;
    private int gnCrudeYawSpin = NOSPIN;

    //Autonomous Constants
    final int AUTON_FWD = 0;
    final int AUTON_BACK = 1;
    final int AUTON_LEFT = 2;
    final int AUTON_RIGHT = 3;
    final int INTERRUPTED = -1;
    final int TIMEDOUT = -2;
    final int LINEFOUND=5;

    //Sensor globals
    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    private final float[] ghsvValues = new float[3];
    private NormalizedColorSensor colorSensor;
    private double dAlphaBackground;


    /**
     * Constructor for  field-centric omni drive AND a robot-centric tank drive robot chassis
     * Mecanum wheels, 4 motors w/encoders, and a Kauai Labs navX required
     */
    public Chassis_KLNavX_6109_v16a() {

    }


    /**
     * Initialize the chassis. Must be called after class creation
     * "Chassis" consists of the motors plus the navX
     * @param  opMode opMode calling this method
     * @param  nReqLayout layout for chassis.  Either CHASSIS_LEFT_FWD  or CHASSIS_RIGHT_FWD
     *                    CHASSIS_LEFT_FWD: left wheels move forward with pos power applied
     *                                      and Right wheels move back with pos power applied
     *                    CHASSIS_RIGHT_FWD: Right wheels move forward with pos power applied
     *                                      and left wheels move back with pos power applied
     * @param strMtrLFName name of Left Front motor in robot config
     * @param strMtrLBName name of left back motor in robot config
     * @param strMtrRFName name of right front motor in robot config
     * @param strMtrRBName name of right back motor in robot config
     * @param strNavXName  name of navX in robot config
     * @param dRobotWeightLbs the weight of the robot in lbs
     * @param dLFPwrAdj Power adj to left front wheel to compensate for uneven weight distribution
     * @param dLBPwrAdj Power adj to left back wheel to compensate for uneven weight distribution
     * @param dRFPwrAdj Power adj to right front wheel to compensate for uneven weight distribution
     * @param dRBPwrAdj Power adj to right back wheel to compensate for uneven weight distribution
     *
     * Note: incorrect requested layout defaults to CHASSIS_LEFT_FWD
     */
    public void initChassis(OpMode opMode, int nReqLayout,String strMtrLFName, String strMtrLBName,
                             String strMtrRFName, String strMtrRBName, String strNavXName,
                             double dRobotWeightLbs, double dLFPwrAdj,double dLBPwrAdj,
                             double dRFPwrAdj,double dRBPwrAdj,String csFront) {
        initNavX(opMode,strNavXName);
        initMotors(opMode, nReqLayout,strMtrLFName,strMtrLBName,strMtrRFName,strMtrRBName);
        gdRobotWeightLbs =dRobotWeightLbs;
        gnControlMode=OMNI_CONTROL;
        gdOmniPwrMagnitude=OMNI_FAST_PWR;
        gdLFPwrAdj=dLFPwrAdj;
        gdLBPwrAdj=dLBPwrAdj;
        gdRFPwrAdj=dRFPwrAdj;
        gdRBPwrAdj=dRBPwrAdj;
        calcMomentumAdjustmentDeg();
        initColorSensor(opMode,csFront);

    }



    /**
     * Close the chassis
     * @param opMode opmode calling this method
     */
    void closeChassis (OpMode opMode) {
        if(isNavXConnected()) enableNavXYawPID(false);
        closeNavX();
    }

    /**
     * init motors based on nReqLayout.
     *    motors must be named mtrLeftFront, mtrLeftBack, mtrRightFront, mtrRightBack
     * @param  opMode used to find chassis motors in robot configuration
     * @param  nReqLayout   Layout requested by user; Either CHASSIS_LEFT_FWD or CHASSIS_RIGHT_FWD
     *       CHASSIS_LEFT_FWD use this when applying positive pwr to left motors drives the left wheels forward
     *          This setting implies right wheels move forward with negative pwr
     *          This setting is the default if an invalid nReqLayout sent
     *          Use this for: NeverRest 20 Orbital motors
     *       CHASSIS_RIGHT_FWD use this when applying positive pwr to right motors drives the right wheels forward
     *          Setting implies left wheels move forward with negative pwr
     *          Use this for: NeverRest 3.7 Orbital motors
     * @param strMtrLFName name used in robot configuration for the left front motor
     * @param strMtrLBName name used in robot configuration for the left back motor
     * @param strMtrRFName name used in robot configuration for the right front motor
     * @param strMtrRBName name used in robot configuration for the right back motor
     * return: none. Sets global variable gnChassisLayout based on parameter nReqLayout
     */
    private void initMotors(OpMode opMode, int nReqLayout,String strMtrLFName,String strMtrLBName,
                            String strMtrRFName, String strMtrRBName) {
        //setup motors
        // Define and Initialize Motors
        //no Try..Catch here... Need to have everything stop if motors cannot be setup
        mtrLeftBack = opMode.hardwareMap.get(DcMotor.class, strMtrLBName);
        mtrRightBack = opMode.hardwareMap.get(DcMotor.class, strMtrRBName);
        mtrLeftFront = opMode.hardwareMap.get(DcMotor.class, strMtrLFName);
        mtrRightFront = opMode.hardwareMap.get(DcMotor.class, strMtrRFName);
        if (nReqLayout == CHASSIS_LEFT_FWD) {
            mtrLeftFront.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            mtrRightFront.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            mtrLeftBack.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates clockwise
            mtrRightBack.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            gnChassisLayout =nReqLayout;
        } else if (nReqLayout == CHASSIS_RIGHT_FWD) {
            //set motors to NeverRest 3.7 Orbital
            mtrLeftFront.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
            mtrRightFront.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates clockwise
            mtrLeftBack.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
            mtrRightBack.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates  clockwise
            gnChassisLayout =nReqLayout;
        } else  { //unsupported requested layout sent... use CHASSIS_LEFT_FWD settings;
            mtrLeftFront.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
            mtrRightFront.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            mtrLeftBack.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
            mtrRightBack.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
            gnChassisLayout =CHASSIS_LEFT_FWD;
        }

        // Stop all chassis motion by setting each axis value to zero
        mtrLeftFront.setPower(0);
        mtrRightFront.setPower(0);
        mtrLeftBack.setPower(0);
        mtrRightBack.setPower(0);
    }

    /**
     * init Kauai Labs navX
     *    navX must be named navx in robot configuration
     * @param  opMode used to find navx in robot configuration
     * @param strNavXName name of navX
     * return: none. Sets global variable gnNavXStatus NAVX_QC_LIB or NAMX_NONE
     *               NAVX_KAUAILABS_LIB  Use Kauai Labs Libraries for orientation
     *               NAVX_QC_LIB  Use FTC/Qualcomm libraries for orientation
     *               NAVX_NONE no navx found
     */
    private boolean initNavX(OpMode opMode,String strNavXName) {
        //setup nav
        opMode.telemetry.addData("Status:", "Setting up "+strNavXName);
        opMode.telemetry.update();


        if(initNavXKL(opMode,strNavXName)==true) {
            //Kauai Labs libraries
            gnNavXStatus = NAVX_KAUAILABS_LIB;
            return true;
        } else {

           //Using FTC/Qualcomm  libraries
            if (initNavXQC(opMode,strNavXName)!=NAVX_QC_LIB) {
                gnNavXStatus = NAVX_ERROR;
                return true;
            } else { //calibration ok
                gnNavXStatus = NAVX_QC_LIB;
                return false;
            }

       }


    }
    private void initColorSensor(OpMode opMode, String strCSName){
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, strCSName);

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        //now lets take a reading. Assume that the sensor is pointing down on a ummarked
        //portion of the mat
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), ghsvValues);
        //store the base color found
        dAlphaBackground= colors.alpha;

    }

     /**
     * main method call for teleop OpModes
     *   Checks if user selected omni or tank control mode for chassis operation
     *      omni control: field oriented movement, robot can rotate while moving
     *      tank control: robots moves fwd, back, left and right OR rotates
     *   Checks throttle settings. omni and tank modes use throttle to set robot speed
     *   Checks if user changed omni or tank mode control options
     *      omni options: left hand driver or right hand driver
     *      tank options: forward mode, or reverse mode (switches front and back of robot)
     *   Moves robot using tank mode or omni mode, based on above settings above
     *
     * Control mapping on gamepad
     *
     *        Triggers and Bumpers change the robot's throttle settings
     *          ==LT==(set throttle slower)      ==RT==(set throttle to slow)
     *          ==LB==(set throttle faster)       ==RB==(Set throttle to fast)
     *         ----------------               --------------
     *       /                 \------------/               \
     *      /                  (tank)   (omni)                \
     *   /            up       Back     Start        Y          \    <---Back and Start Buttons set control mode, tank or omni
     *  /        left   right                     X     B        \        omni mode is field oriented.  robot can move any direction AND spin at same time
     *  |            down                            A           |        tank mode is robot oriented.  robot moves left, right, front, and spins separately
     *  |                                                        |
     *  |              ^                            ^            |
     *  |              |                            |            |
     *  |      <-- Lt Stick -->             <-- Rt Stick -->     |  <---The gamepad sticks are also buttons. this sets the options for omni or tank modes
     *  |              |                            |            |          omni mode:
     *  |              v                            v            |             Lt Stick button pressed: left hand option- Lt Stick is direction, Rt Stick and Y,B button is rotation
     *  |                 /---------------------\                |             Rt Stick button pressed: right hand option-Rt Stick is direction, Lt Stick and left,right  button is rotation
     *  |               /                         \              |          tank mode:
     *  \              /                           \             /             Lt Stick button pressed: Lt Stick moves Lt wheels, Rt Stick moves Rt Wheels, motors forward (forward opt)
     *   \            /                             \           /              Rt Stick button pressed: Rt Stick moves Lt wheels, kkLt Stick moves Rt Wheels, motors reverse (backwards opt)
     *    \ ........./                               \---------/                  Backwards opt is handy when a robot's intake or scoring mechanism is on the back of the robot
     *
     * Buttons near the top of the gamepad control "crawl"
     * Crawl is for robot oriented fine tuning position movement, such as moving slightly left to aim
     * omni mode
     *    left hand option- up,down.left,right is robot direction (robot oriented); X,B spins CW,CCW
     *    right hand option- Y,A,X,B is robot direction (robot oriented), left,right spins CW,CCW
     * tank mode
     *    forward option- Y,A,X,A moves robot fwd,back,left,right; left,right spins CW,CCW
     *    reverse option- A,X,B,X moves robot fwd,back,left,right; right,left spins CW,CCW
     * 
     * @param  gamepad driver's gamepad, typically gamepad1
     * return none
     */
    public void chkmoveChassis(Gamepad gamepad) {
        chkOmniOrTankMode(gamepad);
        chkThrottle(gamepad);
        chkOmniOrTankControlOptions(gamepad);
        if(gnControlMode ==TANK_CONTROL) {
            moveTank(gamepad);
        } else {
            moveOmni(gamepad);
        }
        chkNavXZero(gamepad);
    }

    /**
     * Check if user selects a new control mode: omni or tank
     *    start button pressed: omni mode used
     *     back button pressed: tank mode used
     *
     * @param  gamepad driver's gamepad, typically gamepad1
     * return none.  global variable gnControlMode is set
     */
    private void chkOmniOrTankMode(Gamepad gamepad) {
        if ((System.currentTimeMillis() - glLastControlModePress) < CONTROL_SENSITIVITY) return;

        if(gamepad.start) { //go to Omni mode
            if(gamepad.a) { //check if this a control request... not start-a
                //user hit start-a, don't do anything
                return ;
            }
            glLastControlModePress = System.currentTimeMillis();
            //check if this a mode change, or already in omni mode
            if (gnControlMode != OMNI_CONTROL)
                //not in omni mode, so go in to omni
                gofromTankToOmni();
            gnControlMode =OMNI_CONTROL;
            return;
        }

        if(gamepad.back) { //go to Tank mode
            glLastControlModePress = System.currentTimeMillis();
            //check if this a mode change, or already in tank mode
            if(gnControlMode !=TANK_CONTROL)
                //not in tank mode, go to tank
                gofromOmniToTank();
            gnControlMode =TANK_CONTROL;
            return ;
        }
    }

    /**
     * Check if user wants to change their OPTIONS on their current control mode
     * the control mode is set elsewhere
     * Options in omni control mode
     *      left hand option: left stick controls direction,
     *      user can make right stick control direction, left stick control orientation (right hand drivers)
     * Options in tank control mode
     *      tank options: user can put tank in forward mode, or reverse mode (switches front and back of robot)
     *      Example forward option:user pushes sticks forward, tank moves forward
     *                             left stick controls left wheels
     *                             right stick controls right wheels
     *      Example reverse option:user pushes sticks forward, tank moves backwards,
     *                             left stick controls right wheels
     *                             right stick controls left wheels
     *                             WHY???  sometimes intake modules or scoring modules are on the
     *                             back of the robot. In tank mode, it is hard to drive backwards
     *                             This mode makes the back of the robot the front, so it is easier to drive
     *                             and use the control on the back of the robot
     * @param gamepad gamepad used by driver
     * return none
     *    omni mode:  gbYawStick, gbDirectionStick can be swapped
     *    tank mode:  gnTankSetting set to forward or reverse
     */
    private void chkOmniOrTankControlOptions(Gamepad gamepad ) {
        if ((System.currentTimeMillis() - glLastControlModePress) < CONTROL_SENSITIVITY) return;
        if(gnControlMode == OMNI_CONTROL) {
            //in  Omni
            if(gamepad.left_stick_button) {
                gbDirectionStick = STICK_LEFT;
                gbOrientationStick = STICK_RIGHT;
                glLastControlModePress = System.currentTimeMillis();
            }
            if(gamepad.right_stick_button){
                gbDirectionStick = STICK_RIGHT;
                gbOrientationStick = STICK_LEFT;
                glLastControlModePress = System.currentTimeMillis();
            }
        } else if(gnControlMode ==TANK_CONTROL) {
            //in Tank
            if(gamepad.left_stick_button)  {
                gnTankOption = TANK_OPTION_FWD;
                glLastControlModePress = System.currentTimeMillis();
            }
            if(gamepad.right_stick_button){
                gnTankOption = TANK_OPTION_REV;
                glLastControlModePress = System.currentTimeMillis();
            }
        }
        //else... do nothing!
    }

    /**
     * Move robot in tank mode
     *    robot moves left, right, forward or back
     *
     * @param gamepad driver's gamepad, typically gamepad1
     * return none. global variables gdResultantLFPwr, gdResultantRFPwr, gdResultantLBPwr, gdResultantRBPwr set 
     */
    private void moveTank(Gamepad gamepad) {
        //check if we are in TANK CONTROL
        if (gnControlMode != TANK_CONTROL) return;

        //if got here, in tank control so can set power
        setTankPwr(gamepad);

        //next apply power to each wheel
        mtrLeftFront.setPower(gdResultantLFPwr);  mtrRightFront.setPower(gdResultantRFPwr);
        mtrLeftBack.setPower(gdResultantLBPwr); mtrRightBack.setPower(gdResultantRBPwr);
    }

    /**
     * Move robot in  omni mode
     *    Direction stick moves robot in field oriented direction.
     *    Rotate stick or Buttons rotates the robot
     *    Robot can move in field oriented direction and spin simultaneously
     * @param gamepad gamepad used by driver
     * return none. global variables gdResultantLFPwr, gdResultantRFPwr, gdResultantLBPwr, gdResultantRBPwr set
     */
    private void moveOmni(Gamepad gamepad) {
        //check if in OMNI CONTROL mode
        if(gnControlMode !=OMNI_CONTROL)  return;

        //if got here, in omni control  mode

        //first set power needed for each wheel for omni mode
        setOmniPwr(gamepad, gbDirectionStick, gbOrientationStick);

        //next apply power to each wheel
        apply_omni_pwr();

    }

     /**
      * Apply power to wheels, adjust for uneven weight on each wheel
      * Only adjust for crawl control left-Right movement
      */
     void apply_omni_pwr(){
             mtrRightBack.setPower(gdResultantRBPwr);
             mtrLeftBack.setPower(gdResultantLBPwr);
             mtrLeftFront.setPower(gdResultantLFPwr);
             mtrRightFront.setPower(gdResultantRFPwr);

     }

    /**
     * Check robot throttle setting
     *    Omni mode and Tank mode uses these throttle settings
     *    Right Bumper sets throttle to max speed
     *    Right Trigger sets throttle to min speed
     *    Left Bumper increases speed up to max speed (unless disabled)
     *    Left Trigger reduces speed down to min speed (unless disabled)
     * @param gamepad driver's gamepad, typically gamepad1
     * return: none.  global variables gdTankPwrMagnitude, gdTankMovement,gdOmniPwrMagnitude,gdMomentumAdjustment  set
     */
    private void chkThrottle(Gamepad gamepad) {
        double dCalc;
        if((System.currentTimeMillis()- glLastControlModePress)<CONTROL_SENSITIVITY) return;
        if (gamepad.right_trigger>.5) {//hit right trigger, slow speed setting
            if (gnControlMode == TANK_CONTROL) {
                gdTankPwrMagnitude =TANK_SLOW_PWR;
                gnTankMovement = TANK_NO_MOVEMENT;
            } else if (gnControlMode == OMNI_CONTROL) {
                gdOmniPwrMagnitude = OMNI_SLOW_PWR;
            }
            calcMomentumAdjustmentDeg();
            glLastControlModePress =System.currentTimeMillis();
        } else if (gamepad.right_bumper) {//hit right bumper, fast speed setting
            if(gnControlMode ==TANK_CONTROL) {
                gdTankPwrMagnitude =TANK_FAST_PWR;
                gnTankMovement = TANK_NO_MOVEMENT;
            } else if(gnControlMode ==OMNI_CONTROL)  {
                gdOmniPwrMagnitude =OMNI_FAST_PWR;
            }
            calcMomentumAdjustmentDeg();
            glLastControlModePress =System.currentTimeMillis();
        } else if (gamepad.left_trigger>.5) {//hit left trigger, slow down
            if(gbDontUseLeftTrigBmpr==true) return;
            if (gnControlMode == TANK_CONTROL) {
                dCalc = gdTankPwrMagnitude - TANK_PWR_INTVL;
                if (dCalc >= TANK_SLOW_PWR) {
                    gdTankPwrMagnitude = dCalc;
                    calcMomentumAdjustmentDeg();
                }
                gnTankMovement = TANK_NO_MOVEMENT;
            } else if (gnControlMode == OMNI_CONTROL) {
                dCalc = gdOmniPwrMagnitude - OMNI_PWR_INTVL;
                if (dCalc >= OMNI_SLOW_PWR) {
                    gdOmniPwrMagnitude = dCalc;
                    calcMomentumAdjustmentDeg();
                }
            }
            glLastControlModePress = System.currentTimeMillis();
        }  else if (gamepad.left_bumper) {//hit left bumper, speed up
            if(gbDontUseLeftTrigBmpr==true) return;
            if(gnControlMode ==TANK_CONTROL) {
                dCalc = gdTankPwrMagnitude + TANK_PWR_INTVL;
                if (dCalc <= TANK_FAST_PWR) {
                    gdTankPwrMagnitude = dCalc;
                    calcMomentumAdjustmentDeg();
                }
                gnTankMovement = TANK_NO_MOVEMENT;
            } else if(gnControlMode ==OMNI_CONTROL) {
                dCalc = gdOmniPwrMagnitude + OMNI_PWR_INTVL;
                if(dCalc <= OMNI_FAST_PWR) {
                    gdOmniPwrMagnitude = dCalc;
                    calcMomentumAdjustmentDeg();
                }
            }
            glLastControlModePress =System.currentTimeMillis();
        }
    }

    void chkNavXZero(Gamepad gamepad) {
        if(gbDirectionStick==STICK_LEFT) {
           if(gamepad.y) {
               navX.zeroYaw();
               gdYawAngleToHold = 0;
               gdYawAngleToHold = 0;
           }
        }
        else {
            if(gamepad.dpad_up) {
                navX.zeroYaw();
                gdYawAngleToHold = 0;
            }
        }
    }

     /** Calculate the momentum adjustment to make for robot spinning beyond the point
      *  where user released their controls.
      *  The momentum adjustment is dependent on the robot power and robot weight
      *  The inputs: gdRobotWeightLbs
      * stores result in global variable gdMomemtumAdjDeg
      */

    private void calcMomentumAdjustmentDeg() {
        // 25lb robot slow spin = 5 deg fast sopin =
        //      12|          x
        //  deg   |
        //       8|
        //        |
        //       4|
        //        |    x
        //        + - -+- - +
        //            .5    1
        // y=mx + b
        // calc m:
        //  Slope = (12-2)/(1-.5)
        //       = 20 = m
        // calc b:
        //   12 = 20(1) + b
        //    b = -8
        //parameters to adjust:
        double dMaxMomentumAdj=8; //make this bigger if robot correct CCW after a CW rotation
                                  //make this smaller if robot corrects CW after a CW rotation
        double dMinMomentumAdj=2; //this is probably okay
        double dMaxPwr=1;
        double dMinPwr=.5;
        //calculation on above parameters
        double dAdjDeg;
        double dSlope=(dMaxMomentumAdj-dMinMomentumAdj)/(dMaxPwr-dMinPwr);
        double dIntercept=dMaxMomentumAdj-(dSlope*dMaxPwr);

        //use below for weight compensation
        double dCtlWtLbs = 25d; //these values were based on a 25lbl robot with about the same wt on each wheel

        // first calc deg adjustment given the power
        if(gnControlMode==TANK_CONTROL) dAdjDeg=(dSlope*gdTankPwrMagnitude) + dIntercept;
        else dAdjDeg=(dSlope*gdOmniPwrMagnitude) + dIntercept;

        dAdjDeg=(dSlope*gdOmniPwrMagnitude) + dIntercept;
        //compnesate for weight
        gdMomentumAdjustmentDeg =(gdRobotWeightLbs/dCtlWtLbs)*dAdjDeg;

    }


    /**
     * change control modes from Omni to Tank
     * initialize appropriate variables
     *  
     */
    private void gofromOmniToTank(){
        gnTankMovement =TANK_NO_MOVEMENT;
        gnTankPrevMove =TANK_NO_MOVEMENT;
        setMecWheelDirectionPwrZero();
        gdRotationPwr = 0;
        gbRotationStkOrBtnPressed = false;
        gnRobotSpin =NOSPIN;
        enableNavXYawPID(true);
        setpointNavXYaw(getNavXYaw());
    }

    /**
     * set the power on each wheel needed for tank motion.
     * tank motion has 2 components
     *    direction - robot moves fwd, back, left, right based on gamepad sticks and buttons
     *                Not field oriented, direction is robot oriented, navX not used
     *    rotation -  when moving horiz or vert, or with crawl controls the orientation of the
     *                robot is maintained  by using the navX
     *
     * @param gamepad driver's gamepad, typically gamepad1
     */
    private void setTankPwr(Gamepad gamepad) {
        int nRes;

        //find power to apply to each wheel from the stick positions
        //save the movement before checking current movement
        gnTankPrevMove = gnTankMovement;
        //now set gnTankMovement
        if(setTankStickDirPwr(gamepad)) { //set gnTankMovement when true
            //if got here, one or both sticks were actuated
            //check if stick horizontal movement
            if (gnTankMovement == TANK_HORIZONTAL_MOVEMENT) {
                if (gnTankPrevMove != TANK_HORIZONTAL_MOVEMENT) {
                    //if got here, tank was not moving horiz, and now moving horiz
                    //enable the YawPID controller so that can maintain continued hariz movement
                    enableNavXYawPID(true);
                }
                setTankRotationPwr(gnTankPrevMove, gnTankMovement);
            } else if (gnTankMovement == TANK_VERTICAL_MOVEMENT) {
                if (gnTankPrevMove != TANK_VERTICAL_MOVEMENT) {
                    //if got here, tank was not moving vert, and now moving vert
                    //enable the YawPID controller so that can maintain continued vert movement
                    enableNavXYawPID(true);
                }
                setTankRotationPwr(gnTankPrevMove, gnTankMovement);
            } else {
                //not vertical or horizontal movement if got here
                //turn off PID controller because manually spinning or turning robot
                enableNavXYawPID(false);
                gdRotationPwr = 0; //rotation or turn will come from stick only
            }
            //sticks were not actuated
            //see if buttons pressed for movement
        } else if(setTankCrawlDirPwr(gamepad)) {
            if (gnTankMovement == TANK_HORIZONTAL_MOVEMENT) {
                /*
                if(gnTankPrevMove!=TANK_HORIZONTAL_MOVEMENT) enableNavXYawPID(true);
                setTankRotationPwr(gnTankPrevMove, gnTankMovement);
                if(gnTankPrevMove != TANK_VERTICAL_MOVEMENT) enableNavXYawPID(true);
                setTankRotationPwr(gnTankPrevMove, gnTankMovement);
                 */
                enableNavXYawPID(false);
                setpointNavXYaw(getNavXYaw());
                gdRotationPwr=0;
                gnRobotSpin =NOSPIN;
            }
            else { //tank is moving vertically
                //not vertical or horizontal movement if got here
                //i.e. tank could be doing a loop turn or a pivot turn
                //in this case, no corrective rotational powe
                enableNavXYawPID(false);
                setpointNavXYaw(getNavXYaw());
                gnRobotSpin =NOSPIN;
                gdRotationPwr=0;
            }
        } else {
            //if got here, sticks not pressed, btns not pressed
            gnTankMovement=TANK_NO_MOVEMENT;
            if(gnTankPrevMove!=TANK_NO_MOVEMENT) {
               //tank not moving, before it was
                enableNavXYawPID(true);
            }

            gdDirectionLFPwr = 0;
            gdDirectionLBPwr = 0;
            gdDirectionRFPwr = 0;
            gdDirectionRBPwr = 0;

        }
        //at this point:
        // 1.  Have the power to apply to each wheel to go in the stick/btn direction
        //     while maintaining existing orientation
        //     The power to apply stored in dDirLFPwr, dDirFRPwr, etc)
        // 2.  Have the rotation power to rotate the robot (dRotationPwr)
        //     Adding yaw power to each wheel's directional  power will result correct movement
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another
        addDirectionAndRotationPwr();
    }

    /**
     * Set the power to apply to each wheel to go in robot oriented direction based on rt, lt sticks
     * the directional power for each wheel is set
     *
     * @param gamepad driver's gamepad, typically gamepad1
     * @return true:sticks were actuated, false:sticks were not actuated
     * global variables set: dDirectionLFPwr, dDirectionLBPwr, dDirectionRFPwr, dDirectionRBPwr
     */
    private boolean setTankStickDirPwr(Gamepad gamepad) {

        int nLeftInput=0, nRightInput=0;
        //now apply power
        //check left y
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +

        //check left stick up
        if ((gamepad.left_stick_y < -TANK_STICK_DEADZONE) &&  //left stick moved UP!!
            (gamepad.left_stick_x < TANK_STICK_DEADZONE) &&
            (gamepad.left_stick_x > -TANK_STICK_DEADZONE)) {
            nLeftInput=1;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionLFPwr = gdTankPwrMagnitude; //forward
                gdDirectionLBPwr = gdTankPwrMagnitude; //forward
            } else { //chassis is reversed
                gdDirectionRFPwr =-gdTankPwrMagnitude; //backward
                gdDirectionRBPwr =-gdTankPwrMagnitude; //backward
            }
        }
        //check left stick down
        if ((gamepad.left_stick_y > TANK_STICK_DEADZONE) &&  //left stick moved DOWN!!
            (gamepad.left_stick_x < TANK_STICK_DEADZONE) &&
            (gamepad.left_stick_x > -TANK_STICK_DEADZONE)) {
            nLeftInput=-1;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionLFPwr =-gdTankPwrMagnitude; //backward
                gdDirectionLBPwr =-gdTankPwrMagnitude; //backward
            } else {  //chassis is reversed
                gdDirectionRFPwr = gdTankPwrMagnitude; //forward
                gdDirectionRBPwr = gdTankPwrMagnitude; //forward
            }
        }
        //check right stick up
        if ((gamepad.right_stick_y < -TANK_STICK_DEADZONE) && //right stick moved UP!!
            (gamepad.right_stick_x < TANK_STICK_DEADZONE) &&
            (gamepad.right_stick_x > -TANK_STICK_DEADZONE)) {
            nRightInput=1;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionRFPwr = gdTankPwrMagnitude; //forward
                gdDirectionRBPwr = gdTankPwrMagnitude; //forward
            } else { //chassis is reversed
                gdDirectionLFPwr =-gdTankPwrMagnitude; //backward
                gdDirectionLBPwr =-gdTankPwrMagnitude; //backward
            }
        }
        //check right stick down
        if ((gamepad.right_stick_y > TANK_STICK_DEADZONE) &&  //right stick moved DOWN
            (gamepad.right_stick_x < TANK_STICK_DEADZONE) &&
            (gamepad.right_stick_x > -TANK_STICK_DEADZONE)) {
            nRightInput=-1;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionRFPwr =-gdTankPwrMagnitude; //backward
                gdDirectionRBPwr =-gdTankPwrMagnitude; //backward
            } else { //chassis is reversed
                gdDirectionLFPwr = gdTankPwrMagnitude; //forward
                gdDirectionLBPwr = gdTankPwrMagnitude; //forward
            }
        }

        //now check left-right horizontal movement
        //check for move left  (both sticks left)
        if ((gamepad.left_stick_x < -TANK_STICK_DEADZONE) &&  //left stick moved LEFT
            (gamepad.left_stick_y > -TANK_STICK_DEADZONE)  &&
            (gamepad.left_stick_y < TANK_STICK_DEADZONE) &&
            (gamepad.right_stick_x < -TANK_STICK_DEADZONE) &&  //right stick moved LEFT
            (gamepad.right_stick_y > -TANK_STICK_DEADZONE) &&
            (gamepad.right_stick_y < TANK_STICK_DEADZONE)) {
            nRightInput=-2;
            nLeftInput=-2;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionLFPwr = -gdTankPwrMagnitude;
                gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;
                gdDirectionRBPwr = -gdTankPwrMagnitude;
            } else { //chassis is reversed so move right
                gdDirectionLFPwr = gdTankPwrMagnitude;
                gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;
                gdDirectionRBPwr = gdTankPwrMagnitude;
            }
        }

        //check for move right (both sticks right)
        if ((gamepad.left_stick_x > TANK_STICK_DEADZONE) &&  //left stick moved RIGHT
                (gamepad.left_stick_y > -TANK_STICK_DEADZONE) &&
                (gamepad.left_stick_y < TANK_STICK_DEADZONE) &&
                (gamepad.right_stick_x > TANK_STICK_DEADZONE) &&  //right stick moved RIGHT
                (gamepad.right_stick_y > -TANK_STICK_DEADZONE) &&
                (gamepad.right_stick_y < TANK_STICK_DEADZONE)) {
            nRightInput=2;
            nLeftInput=2;
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionLFPwr = gdTankPwrMagnitude;
                gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;
                gdDirectionRBPwr = gdTankPwrMagnitude;
            } else { //chassis is reversed so move left
                gdDirectionLFPwr = -gdTankPwrMagnitude;
                gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;
                gdDirectionRBPwr = -gdTankPwrMagnitude;
            }
        }

        //now check if sticks were moved.  turn off motors it they were not
        if (nLeftInput==0) {
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionLFPwr = 0;
                gdDirectionLBPwr = 0;
            } else { //chassis is reversed
                gdDirectionRFPwr = 0;
                gdDirectionRBPwr = 0;
            }
        }
        if (nRightInput==0) {
            if (gnTankOption == TANK_OPTION_FWD) {
                gdDirectionRFPwr = 0;
                gdDirectionRBPwr = 0;
            } else {
                gdDirectionLFPwr = 0;
                gdDirectionLBPwr = 0;
            }
        }

        //set tank movement based on sticks
        if((nLeftInput==1) && (nRightInput==1)) {
            gnTankMovement=TANK_VERTICAL_MOVEMENT;
            gnRobotSpin=NOSPIN;
        }
        else if((nLeftInput==-1) && (nRightInput==-1)){
            gnTankMovement=TANK_VERTICAL_MOVEMENT;
            gnRobotSpin=NOSPIN;
        }
        else if((nLeftInput==2) && (nRightInput==2)) {
            gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
            gnRobotSpin=NOSPIN;
        }
        else if((nLeftInput==2) && (nRightInput==2)) {
            gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
            gnRobotSpin=NOSPIN;
        }
        else if((nLeftInput==1) && (nRightInput==-1)) {
            gnTankMovement= TANK_SPIN_MOVEMENT;
            gnRobotSpin=CW;
        }
        else if((nLeftInput==-1) && (nRightInput==1)) {
            gnTankMovement= TANK_SPIN_MOVEMENT;
            gnRobotSpin=CCW;
        } else if((nLeftInput!=0)&&((nRightInput==1)||(nRightInput==-1))) {
            gnTankMovement=TANK_TURN_MOVEMENT;
            gnRobotSpin=NOSPIN;
        } else if((nRightInput!=0)&&((nLeftInput==1)||(nLeftInput==-1))) {
            gnTankMovement=TANK_TURN_MOVEMENT;
            gnRobotSpin=NOSPIN;
        }

        if((nLeftInput==0) &&(nRightInput==0)) return false;
        else return true;
    }

    /**
     * set the crawl direction power for tank control  each mecanum wheel
     * "crawling" is moving the robot in a robot oriented direction fwd, back,lefi,right
     * why would you want to do this???
     *    Say robot is at a 57 degree angle and the game element is directly in front
     *    It is very difficult to move the both sticks fwd at exactly the same time
     *    Instead, the driver can simply press the crawl forward button to move the robot forward
     * The crawl buttons are A,B,X,Y
     *             dpad                  buttons
     *              Up                      Y
     *          Left  Right             X       B
     *             Down                     A
     * If the tank forward option is set
     *              Up                        Y (fwd)
     *      Left(CCW) Right(CW)         X(left)   B(right)
     *             Down                       A (back)
     * If the tank reverse option is set (back of robot is treated as the front
     *              Up                        Y (back)
     *      Left(CW)  Right(CCW)        X(right)    B(left)
     *             Down                       A (fwd)
     *
     * @param gamepad driver's gamepad, typically gamepad1
     */
    private boolean setTankCrawlDirPwr(Gamepad gamepad) {
        //The directional crawl buttons are above the right stick
        if(gnTankOption == TANK_OPTION_FWD) {
        //   If the gamepad left stick is the direction stick, the crawl buttons are dpad buttons
        //   If the gamepad right stick is the direction stick, the crawl buttons are A,B,X,Y
            if(gamepad.y) {
                //apply pwr fwd
                gdDirectionLFPwr = gdTankPwrMagnitude;  gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;  gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement= TANK_VERTICAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.a) {
                //apply pwr back
                gdDirectionLFPwr = -gdTankPwrMagnitude;  gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;  gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement= TANK_VERTICAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.b) {
                //move right
                gdDirectionLFPwr = gdTankPwrMagnitude;  gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;  gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.x) {
                //move left
                gdDirectionLFPwr = -gdTankPwrMagnitude;  gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;  gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.dpad_left) {
                //rotate CCW
                gdDirectionLFPwr = -gdTankPwrMagnitude;  gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;  gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement= TANK_SPIN_MOVEMENT;
                gnRobotSpin=CCW;
                return true;
            }
            else if(gamepad.dpad_right) {
                //rotate CW
                gdDirectionLFPwr = gdTankPwrMagnitude; gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude; gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement = TANK_SPIN_MOVEMENT;
                gnRobotSpin=CW;
                return true;
            }
        } else { //TANK_OPTION_REV
            if(gamepad.a) {
                //apply pwr fwd
                gdDirectionLFPwr = gdTankPwrMagnitude;  gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;  gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement= TANK_VERTICAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            } else if(gamepad.y) {
                //apply pwr back
                gdDirectionLFPwr = -gdTankPwrMagnitude;  gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;  gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement= TANK_VERTICAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.x) {
                //move right
                gdDirectionLFPwr = gdTankPwrMagnitude;  gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude;  gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.b) {
                //move left
                gdDirectionLFPwr = -gdTankPwrMagnitude;  gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;  gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement=TANK_HORIZONTAL_MOVEMENT;
                gnRobotSpin=NOSPIN;
                return true;
            }
            else if(gamepad.dpad_right) {
                //rotate CW
                gdDirectionLFPwr = gdTankPwrMagnitude;  gdDirectionRFPwr = -gdTankPwrMagnitude;
                gdDirectionLBPwr = gdTankPwrMagnitude;  gdDirectionRBPwr = -gdTankPwrMagnitude;
                gnTankMovement= TANK_SPIN_MOVEMENT;
                gnRobotSpin=CW;
                return true;
            }
            else if(gamepad.dpad_left) {
                //rotate CCW
                gdDirectionLFPwr = -gdTankPwrMagnitude; gdDirectionRFPwr = gdTankPwrMagnitude;
                gdDirectionLBPwr = -gdTankPwrMagnitude; gdDirectionRBPwr = gdTankPwrMagnitude;
                gnTankMovement = TANK_SPIN_MOVEMENT;
                gnRobotSpin=CCW;
                return true;
            }
        }
        //if got here, nothing pressed
        return false;
    }

    /**
     * Set rotational power to keep robot pointed in the same direction when in tank Control mode
     * NOTE that for TANK movement, the rotational power is correctional only, unlike OMNI movement
     * In tank movement, rotating using the sticks or buttons can be done with the gdDirectionalXXPwr
     * In omni movement cannot rotate robot with the sticks or buttons, so omni rotational power can be correctional OR using yawpid
     *
     * Tank Rotational power (which is correctional) is set as tank moved horiz or vert
     * Tank Rotational power is also set when the tank is not moving
     * Target orientation is set when tank first goes to horiz or first goes to Vert
     * Target orientation is also set when tank is not moving, and had previously moved
     * Special case when not moving, going horz, or vert
     *    and previoulsy spinning:  target orientation set w/momentum adj
     *
     * @param nPrev the previous movement of the tank
     * @param nCurrent the current movement of the tank
     * set global variable gdRotationPwr
     */
    private void setTankRotationPwr(int nPrev, int nCurrent) {

        if(gnControlMode!=TANK_CONTROL) return;

        if(!isNavXConnected()) {
            gdRotationPwr =0;
            return;
        }

        //get correctional power when tank moving horizontally
        if(nCurrent==TANK_HORIZONTAL_MOVEMENT) {
            //set target heading the when the driver starts horizontal or vertical movement;
            if(nPrev!=TANK_HORIZONTAL_MOVEMENT){
                //if got here horizontal  motion was just started, set direction to keep
                //special case  1st
                if(nPrev==TANK_SPIN_MOVEMENT) {
                    //if got here, previously spinning, so adjust for momentum
                    if(gnRobotSpin==CW)  setpointNavXYaw(getNavXYaw()+gdMomentumAdjustmentDeg);
                    else setpointNavXYaw(getNavXYaw()-gdMomentumAdjustmentDeg);
                }
                //normal case
                else {
                    setpointNavXYaw(getNavXYaw());
                }
            }
            //not spinning or turning, so Rotational Pwr is the correctional pwr
            gdRotationPwr = getNavXCorrectionPwr();
            return;
        }

        //get correctional power when tank moving vertically
        if (nCurrent== TANK_VERTICAL_MOVEMENT) {
            //set target heading the when the driver starts horizontal or vertical movement;
            if(nPrev!=TANK_VERTICAL_MOVEMENT) {
                //if got here vertial  motion was just started, set direction to keep
                //special case  1st
                if(nPrev==TANK_SPIN_MOVEMENT) {
                    //if got here, previously spinning, so adjust for momentum
                    if(gnRobotSpin==CW)  setpointNavXYaw(getNavXYaw()+gdMomentumAdjustmentDeg);
                    else setpointNavXYaw(getNavXYaw()-gdMomentumAdjustmentDeg);
                }
                //normal case
                else {
                    setpointNavXYaw(getNavXYaw());
                }
            }
            //not spinning or turning, so Rotational Pwr is the correctional pwr
            gdRotationPwr = getNavXCorrectionPwr();
            return;
        }

        //get correctional power when tank is not moving
        if(nCurrent==TANK_NO_MOVEMENT) {
            //set target heading when driver stops tank movement
            if(nPrev!=TANK_NO_MOVEMENT) {
                //special case  1st
                if(nPrev==TANK_SPIN_MOVEMENT) {
                    //if got here, previously spinning, so adjust for momentum
                    if(gnRobotSpin==CW)  setpointNavXYaw(getNavXYaw()+gdMomentumAdjustmentDeg);
                    else setpointNavXYaw(getNavXYaw()-gdMomentumAdjustmentDeg);
                }
                //normal case
                else {
                    setpointNavXYaw(getNavXYaw());
                }
            }
            //not spinning or turning, so Rotational Pwr is the correctional pwr
            gdRotationPwr = getNavXCorrectionPwr();
            return;
        }

        //all other cases of current tank movement
        gdRotationPwr =0;

    }


    /**
     *  Go from Tank mode to Omni monde
     *     set appropriate global variables to start Omni mode
     *
     */

    private void gofromTankToOmni() {
        gnControlMode = OMNI_CONTROL;
        gnTankMovement =TANK_NO_MOVEMENT;
        gnTankPrevMove =TANK_NO_MOVEMENT;
        setMecWheelDirectionPwrZero();
        gdRotationPwr = 0;
        gbRotationStkOrBtnPressed = false;
        gnRobotSpin =NOSPIN;

        enableNavXYawPID(true);
        setpointNavXYaw(getNavXYaw());
    }


    /**
     * Set the power on each wheel needed for omni motion.
     * Omni motion has 2 components
     *    direction - robot moves in the field oriented direction of the direction stick
     *    rotation - robot rotates clock wise (CW) or counter clock wise (CCW)
     * Omni motion moves the robot in a field oriented direction and can rotate the robot
     *    at the same time
     * @param gamepad driver's gamepad, typically gamepad1
     * @param bDirectionStick STICK_LEFT or STICK_RIGHT (opposite of bRotateStick)
     * @param bRotateStick STICK_RIGHT or STICK_LEFT (opposite of bDirectionStick)
     */
    private void setOmniPwr(Gamepad gamepad, boolean bDirectionStick, boolean bRotateStick) {


        //set power to apply to each wheel to move robot in field oriented direction,
        //    based on direction stick or crawl buttons
        setOmniDirectionPwr(gamepad, bDirectionStick);

        //set rotation power to apply to each wheel rotate robot, based on rotate stick
        setOmniRotationPwr(gamepad, bRotateStick);

        //at this point:
        // 1.  Have the power to apply to each wheel to go in the direction stick (or crawl buttons) direction while maintaining existing yaw
        //     This direction power to apply is stored in dDirLFPwr, dDirFRPwr, etc)
        // 2.  Have the power to rotate the robot (dYawPwr), that adds/subs from existing yaw
        //Adding yaw power to each wheel's directional  power will result in a simultaneous spin and move
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another
        addDirectionAndRotationPwr();
    }

    /**
     * Add directional power and rotational power to get a resultant power to apply to each wheel
     * Applying different amounts of power to each wheel controls robot's movement and spin
     * This allows robot to move in a direction and ratate at the same time
     * The resultant powers may be greater than 1.  If so, scale back powers so tha highest
     *    power is not larger than 1
     */

    private void addDirectionAndRotationPwr() {
        //Adding yaw power to each wheel's directional  power will result in a simultaneous spin and move
        //The sum of these two powers may differ on each wheel, and may exceed 1.  If the sum exceeds
        //1, need to scale down each wheel's power so they keep the same proportional power with
        //respect to one another.  In this case, the highest wheel power will be 1.
        int LF=0, RF=1, LB=2, RB=3;
        int nIdx;

        double[] adResultantPwr;
        adResultantPwr=new double[4];
        double dHighestPwr, dLowestPwr, dScalar;

        adResultantPwr[LF]= gdDirectionLFPwr + gdRotationPwr; adResultantPwr[RF]= (gdDirectionRFPwr) - gdRotationPwr;
        adResultantPwr[LB]= gdDirectionLBPwr + gdRotationPwr; adResultantPwr[RB]= gdDirectionRBPwr - gdRotationPwr;

        dHighestPwr=adResultantPwr[0];
        dLowestPwr=adResultantPwr[0];
        //find highest and lowest valuej
        for(nIdx=0;nIdx<4;nIdx++)
            if(adResultantPwr[nIdx] > dHighestPwr) dHighestPwr=adResultantPwr[nIdx];
            else if(adResultantPwr[nIdx] < dLowestPwr)
                dLowestPwr=adResultantPwr[nIdx];

        //get the largest magnitude, store in dScalar:
        if(Math.abs(dHighestPwr)>Math.abs(dLowestPwr)) dScalar=Math.abs(dHighestPwr );
        else dScalar=Math.abs(dLowestPwr);

        //see if sum of yaw and direction (stored in dScalar) is greater than 1
        //  if the scalsr is greater than 1, need to scale all the wheel's power down until
        //  the point that the highest power is 1, since can't setpower() to more than 1
        if(dScalar>(double)1) {
            //if got here the sum of yaw and direction is too large
            //must scale all values down by the same amount
            for(nIdx=0;nIdx<4;nIdx++) {
               adResultantPwr[nIdx] = adResultantPwr[nIdx] / dScalar;
            }
        }

        gdResultantLFPwr =adResultantPwr[LF]; gdResultantRFPwr =adResultantPwr[RF];
        gdResultantLBPwr =adResultantPwr[LB]; gdResultantRBPwr =adResultantPwr[RB];
    }

    /**
     * Set the power to apply to each wheel to go in field oriented direction of the direction stick
     * What is a field oriented??
     *   Say the robot is pointing right as some random angle
     *   The direction stick is pressed forward
     *   The robot will move away from the driver and keep its random angle to the right
     *   The robot will NOT move right
     *
     * @param gamepad driver's gamepad, typically gamepad1
     * @param bDirStick STICK_LEFT for left handed operation, STICK_RIGHT for right handed operation
     * global variables set: dDirLFPwr, dDirLBPwr, dDirRFPwr, dDirRBPwr
     */
    private  void setOmniDirectionPwr(Gamepad gamepad, boolean bDirStick) {
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
        //
        double dPolar;

        //first get the polar angle of the direction stick
        if((dPolar= calcDirectionStickPolar(gamepad,bDirStick))!=STICK_IN_DEADZONE) {
            //If got here got an input from the direction stick (not in dead zone)
            //Add the robot yaw angle to the direction stick angle, store in dRobotDirectionPolar
            //use the imu Yaw and the dDirectionStickPolar found above
            gdRobotDirectionPolar = setRobotDirectionPolar();
            //From the robot's perspective, moving in dRobotDirectionPolar direction will
            //move the robot in the stick direction, while keeping its yaw (the robot may not have been pointing straight fwd)
            //set the mec wheel power to move in this direction
            //note that this calc is for the robot's existing yaw, not considering the yaw stick yet
            setMecWheelDirPwr(gdRobotDirectionPolar);
            gdDirectionStickPolar =dPolar; //set for other methods
            gnDirInputLast=DIR_LAST_STICK;
            return;
        }
        //if got here stick is in dead zone,
        //see if direction buttons pressed
        if(setOmniCrawlPwr(gamepad,bDirStick) ==true) {
            //if got here, the crawl buttons were pressed
            //check if this is the first time the crawl buttons were pressed
            if(gnDirInputLast!=DIR_LAST_BTN)  {
                //if got here this is the first time the button was pressed
                //turn off rotation power
                //setpointNavXYaw(getNavXYaw()); 2021/2/8
                gdRotationPwr =0;
                gnRobotSpin =NOSPIN;
                gbRotationStkOrBtnPressed =false; //so can properly handle the rotation controls
            }
            gdDirectionStickPolar =dPolar; //set for other methods
            gnDirInputLast=DIR_LAST_BTN;
            return;
        }

        //no input from the direction stick (in dead zone) and crawl
        gdDirectionStickPolar =dPolar; //set for other methods
        gnDirInputLast=DIR_LAST_NONE;
        setMecWheelDirectionPwrZero();
        return;
    }

    /**
     * Calculate the polar angle of the direction stick
     * @param gamepad driver's gamepad, typically gamepad1
     * @param bDirStick STICK_LEFT for left handed operation, STICK_RIGHT for right handed operation
     * @return polar angel of direction stick
     */
    private double calcDirectionStickPolar(Gamepad gamepad, boolean bDirStick) {
        double dStickX, dStickY;
        double dAngle,dPolar;


        //calculate and correct stick's polar angle

        if(chkStickDeadZone(gamepad,bDirStick)) {
            //do not move robot in any direction from the direction stick
            //   robot may still spin from the yaw output
            gdDirectionStickPolar = 0;
            return STICK_IN_DEADZONE;
        }

        //if got here, the direction stick is pressed
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
        if(bDirStick== STICK_LEFT) {
            //check if stick is in dead zone
            dStickX=(double)gamepad.left_stick_x;
            if(dStickX==0) dStickX=.000001; //so don't divide by 0
            dStickY=-(double)gamepad.left_stick_y;
        }
        else { //STICK_RIGHT
            dStickX=(double)gamepad.right_stick_x;
            if(dStickX==0) dStickX=.000001; //so don't divide by 0
            dStickY=-(double)gamepad.right_stick_y;
        }

        dAngle=Math.toDegrees(Math.atan(dStickY/dStickX));
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
        if(dStickX>=0) {
            if(dStickY>=0) {
                dPolar=(double)90-dAngle;
            }
            else { //(dStickY<0)
                dPolar=-dAngle+(double)90;
            }
        }
        else { //dStickX<0
            if(dStickY>=0) {
                dPolar=-(dAngle+(double)90);
            }
            else { //(dStickY<0)
                dPolar=-(dAngle+(double)90);
            }
        }

        return dPolar;
    }


     /**
      * Set the direction to move the robot so that:
      *    1) It travels in the direction of the field oriented direction stick
      *    2) It keeps its current yaw angle
      * @return the polar direction for robot to move from robot's perspective
      */
    private double setRobotDirectionPolar(){
        //From the robot's perspective,it is positioned in at the IMU yaw angle.
        //The direction stick moves the robot in the field's angle, not the robot's angle
        //Therefore: must combine these angles so robot moves in direction of direction stick
        //    while maintaining its yaw angle
        //The combination gives the angle, from the robot's perspective, that it must travel.

        //dDirectionStickPolar  goes between 0 to -180 and 0 to 180
        //dRobotYawAngle goes between 0 to -180 and 0 to 180

        double dRobotYawPolar = getNavXYaw();

        double dChk;

        if((dRobotYawPolar >=0)&&(dRobotYawPolar <=180)) {
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
        //            /   /   | robot is facing +45 deg polar with respect to the field,
        //            \  /    |    so the IMU yaw angle will return +45 polar
        //             \/     |
        //        ------------+----------
        //
        //   Together
        //              /\ Front
        //             /  \   |
        //            /   / ->| From the robot's perspective it must travel 90-45=45 deg polar
        //            \  /    |    this will maintain its yaw, and go in the stick direction
        //             \/     |
        //        ------------+----------
            //dRobotDirectionPolar = dDirectionStickPolar - dRobotYaw;
            dChk = gdDirectionStickPolar - dRobotYawPolar;
            if(dChk<-180d) {
                gdRobotDirectionPolar =dChk+360;
            } else {
                gdRobotDirectionPolar =dChk;
            }
        }
        else { //robot facing -.0~1 to -180
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
            dChk= gdDirectionStickPolar - dRobotYawPolar;
            if(dChk>180) {
                gdRobotDirectionPolar = -(360 - dChk);
            } else {
                gdRobotDirectionPolar = dChk;
            }
            //                             logic
            //DirStick   RobotYaw    dChk  tst    Res
            //  90           0        90           90
            //  90          90         0            0
            //  90         180       -90          -90
            //  90        -180       270  >180    -(360-270)=-90
            //  90         -90       180          180
        }
        return gdRobotDirectionPolar; //setting this global variable above, and returning it
    }

     /**
      * Set each power to each mecanum wheel so that the robot travels in the dRobotPolarTarget
      * direction
      * @param dRobotPolarTarget The target direction to move robot
      */
    private void setMecWheelDirPwr(double dRobotPolarTarget){
        //This method sets each mecanum wheel's power so that the robot travels in the
        //direction of the passed value (dRobotPolarTarget)
        double dFullPwr= gdOmniPwrMagnitude;
        double dScale;


        //correct input
        if(dRobotPolarTarget < -180) {
            //need to go right
            dRobotPolarTarget = (double) 360 + dRobotPolarTarget;
        }
        else if (dRobotPolarTarget > 180){
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

        if((dRobotPolarTarget >=0)&&(dRobotPolarTarget <90)) {
            //for this target polar angle range, LF wheel is always fwd
            gdDirectionLFPwr = dFullPwr;
        }
        else if((dRobotPolarTarget <0)&&(dRobotPolarTarget >=-90)) {
            //for this target polar angle range, LF wheel goes from fwd to back
            //-90----- -45 ------ 0   polar angle
            //-100------ 0 ------ 100  %full pwr
            dScale=(double)1/(double)45; //because 45 is the zero power
            gdDirectionLFPwr =dFullPwr*((double)1+(dRobotPolarTarget *dScale));
            //polar 0 to -45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 + (0 * (1/45))
            //                    fullpwr * (1 + 0) = fullpwr = fullpwr fwd
            //   when target=-45: fullpwr * (1 + (-45 * (1/45))
            //                    fullpwr * (1 + -1) = 0 stop
            //polar -45 to -90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 + (-90 * (1/45))
            //                    fullpwr * (1 + -2) = fullpwr * -1 = -fullpwr = fullpwr back
        }
        else if((dRobotPolarTarget <-90)&&(dRobotPolarTarget >=-180)) {
            //for this target polar angle range, LF wheel is always back
            gdDirectionLFPwr = -dFullPwr;
        }
        else if((dRobotPolarTarget >=90)&&(dRobotPolarTarget <=180)) {
            //for this target polar angle range, LF wheel goes from fwd to back
            //90----- 135 ----- 180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionLFPwr =dFullPwr*((double)3-(dRobotPolarTarget *dScale));
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

        if((dRobotPolarTarget >=0)&&(dRobotPolarTarget <90)) {
            //for this target polar angle range, RF wheel goes from fwd to back
            //0---------45--------90   polar angle
            //100------ 0------- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionRFPwr =dFullPwr*((double)1-(dRobotPolarTarget *dScale));
            //polar 0 to 45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 - (0 * (1/45))
            //                    fullpwr * (1-0) = fullpwr = fullpwr fwd
            //   when target=45:  fullpwr * (1 - (45 * (1/45))
            //                    fullpwr * (1-1) = 0 stop
            //polar 45 to 90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 - (90 * (1/45))
            //                    fullpwr * (1-2) = fullpwr * -1 = -fullpwr = fullpwr back
        }
        else if((dRobotPolarTarget <0)&&(dRobotPolarTarget >=-90)) {
            //for this target polar angle range, RF wheel is always fwd
            gdDirectionRFPwr = dFullPwr;
        }
        else if((dRobotPolarTarget <-90)&&(dRobotPolarTarget >=-180)) {
            //for this target polar angle range, RF wheel goes from fwd to back
            //-90---- -135 ---- -180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionRFPwr =dFullPwr*((double)3+(dRobotPolarTarget *dScale));
            //polar -90 to -135, wheel %pwr will go from 100 to 0
            //   when target=-90: fullpwr * (3 + (-90 * (1/45))
            //                    fullpwr * (3 + (-2)) = fullpwr * 1 = fullpwr fwd
            //   when target=-135:fullpwr * (3 + (-135* (1/45))
            //                    fullpwr * (3 + (-3)) = 0 stop
            //polar -90 to -180, above will go from 0 to -1 because
            //   when target=-180:fullpwr * (3 + (-180 * (1/45))
            //                    fullpwr * (3 + (-4)) = fullpwr * -1 = fullpwr back
        }
        else if((dRobotPolarTarget >=90)&&(dRobotPolarTarget <=180)) {
            //for this target polar angle range, RF wheel is always back
            gdDirectionRFPwr = -dFullPwr;
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

        if((dRobotPolarTarget >=0)&&(dRobotPolarTarget <90)) {
            //for this target polar angle range, LB wheel goes from fwd to back
            //0---------45--------90   polar angle
            //100------ 0------- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionLBPwr =dFullPwr*((double)1-(dRobotPolarTarget *dScale));
            //polar 0 to 45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 - (0 * (1/45))
            //                    fullpwr * (1-0) = fullpwr = fullpwr fwd
            //   when target=45:  fullpwr * (1 - (45 * (1/45))
            //                    fullpwr * (1-1) = 0 stop
            //polar 45 to 90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 - (90 * (1/45))
            //                    fullpwr * (1-2) = fullpwr * -1 = -fullpwr = fullpwr back
        }
        else if((dRobotPolarTarget <0)&&(dRobotPolarTarget >=-90)) {
            //for this target polar angle range, LB wheel is always fwd
            gdDirectionLBPwr = dFullPwr;
        }
        else if((dRobotPolarTarget <-90)&&(dRobotPolarTarget >=-180)) {
            //for this target polar angle range, LB wheel goes from fwd to back
            //-90---- -135 ---- -180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionLBPwr =dFullPwr*((double)3+(dRobotPolarTarget *dScale));
            //polar -90 to -135, wheel %pwr will go from 100 to 0
            //   when target=-90: fullpwr * (3 + (-90 * (1/45))
            //                    fullpwr * (3 + (-2)) = fullpwr * 1 = fullpwr fwd
            //   when target=-135:fullpwr * (3 + (-135* (1/45))
            //                    fullpwr * (3 + (-3)) = 0 stop
            //polar -90 to -180, above will go from 0 to -1 because
            //   when target=-180:fullpwr * (3 + (-180 * (1/45))
            //                    fullpwr * (3 + (-4)) = fullpwr * -1 = fullpwr back
        }
        else if((dRobotPolarTarget >=90)&&(dRobotPolarTarget <=180)) {
            //for this target polar angle range, LB wheel is always back
            gdDirectionLBPwr = -dFullPwr;
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

        if((dRobotPolarTarget >=0)&&(dRobotPolarTarget <90)) {
            //for this target polar angle range, RB wheel is always fwd
            gdDirectionRBPwr = dFullPwr;
        }
        else if((dRobotPolarTarget <0)&&(dRobotPolarTarget >=-90)) {
            //for this target polar angle range, RB wheel goes from fwd to back
            //-90----- -45 ------ 0   polar angle
            //-100------ 0 ------ 100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionRBPwr =dFullPwr*((double)1+(dRobotPolarTarget *dScale));
            //polar 0 to -45, wheel %pwr will go from 100 to 0
            //   when target=0:   fullpwr * (1 + (0 * (1/45))
            //                    fullpwr * (1 + 0) = fullpwr = fullpwr fwd
            //   when target=-45: fullpwr * (1 + (-45 * (1/45))
            //                    fullpwr * (1 + -1) = 0 stop
            //polar -45 to -90, above will go from 0 to -1 because
            //   when target=90:  fullpwr * (1 + (-90 * (1/45))
            //                    fullpwr * (1 + -2) = fullpwr * -1 = -fullpwr = fullpwr back
        }
        else if((dRobotPolarTarget <-90)&&(dRobotPolarTarget >=-180)) {
            //for this target polar angle range, RB wheel is always back
            gdDirectionRBPwr = -dFullPwr;
        }
        else if((dRobotPolarTarget >=90)&&(dRobotPolarTarget <=180)) {
            //for this target polar angle range, RB wheel goes from fwd to back
            //90----- 135 ----- 180  polar angle
            //100------ 0 ----- -100  %full pwr
            dScale=(double)1/(double)45;
            gdDirectionRBPwr =dFullPwr*((double)3-(dRobotPolarTarget *dScale));
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

    /**
     * Set the omni rotation power, based on the rotation stick position
     *    or based on pushing the rotation buttons
     * @param gamepad driver's gamepad, typically gamepad1
     * @param bRotationStick STICK_RIGHT for left handed operation, STICK_LEFT for right handed operation
     */
    private void setOmniRotationPwr(Gamepad gamepad, boolean bRotationStick) {
        //Yaw stick rotates the robot more clockwise or counter clockwise from its current position
        //1.  When the yaw stick is pressed (not in dead zone) rotate robot
        //2.  When the stick is in the dead zone (not pressed), the user wants the robot to stop
        //    turning.  Use the NavX Yaw PID controller (or equivalent) 
        //    to hold the position set when the yaw stick is first released
        //    Note:  The rotot will continue to rotate beyond the point that the user releases the
        //    stick because of momentum.  Therefore, set the yaw to hold slightly beyond the point
        //    that the user releases the stick
        //   

        double dStickX;
        double dMomentumCompensation=0;


        //set Yaw according to stick and buttons
        //check if yaw stick is released
        if (!chkStickDeadZone(gamepad, bRotationStick)) {
            //not in dead zone
            //yaw stick IS now pressed:
            //   1.  yaw pid hold setting needs to change. so disable yaw pid controller;
            //   2.  need to spin robot without yaw pid controller, will enable again when yaw stick released
            //SkyStone v5.2 incompatibility..Fixed!
            enableNavXYawPID(false);
            ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
            //     -          -
            //  -  L  +    -  R
            //     +          +
            if (bRotationStick == STICK_LEFT)
                dStickX = (double) gamepad.left_stick_x;
            else
                dStickX = (double) gamepad.right_stick_x;

            //when dStickX is negative, need to rotate counter clock wise
            if (dStickX < 0) {
                gdRotationPwr = -(gdOmniPwrMagnitude * ROTATION_PWR_RATIO);
                gnRobotSpin = CCW;
                gbRotationStkOrBtnPressed = true; //set so know what happened when this was run
            } else if (dStickX > 0) {
                gdRotationPwr = gdOmniPwrMagnitude * ROTATION_PWR_RATIO;
                gnRobotSpin = CW;
                gbRotationStkOrBtnPressed = true; //set so know what happened when this was run
            }
            return;
        }
        //if got here, rotation stick is in the dead zone, check buttons for rotation
        //check if dpad or buttons were pressed
            //
            //       Up                      Y
            //   Left  Right             X       B
            //      Down                     A
            //
            //     L-Stick               R-Stick
        if(bRotationStick==STICK_LEFT) {
            //if rotation stick is left, rotation buttons on dpad above left stick
            if(gamepad.dpad_left) {
                gdRotationPwr = -(gdOmniPwrMagnitude * ROTATION_PWR_RATIO);
                gnRobotSpin = CCW;
                gbRotationStkOrBtnPressed =true; //set so know what happened when this was run
                return;
            //if rotation stick is left, rotation buttons on dpad above left stick
            } else if(gamepad.dpad_right) {
                gdRotationPwr = gdOmniPwrMagnitude * ROTATION_PWR_RATIO;
                gnRobotSpin = CW;
                gbRotationStkOrBtnPressed =true; //set so know what happened when this was run
                return;
            }
        } else if(bRotationStick==STICK_RIGHT) {
            //if yaw stick is right, yaw buttons are above right stick
            if(gamepad.x) { //left button
                gdRotationPwr = -(gdOmniPwrMagnitude * ROTATION_PWR_RATIO);
                gnRobotSpin = CCW;
                gbRotationStkOrBtnPressed =true; //set so know what happened when this was run
                return;
            } else if(gamepad.b) {
                gdRotationPwr = gdOmniPwrMagnitude * ROTATION_PWR_RATIO;
                gnRobotSpin = CW;
                gbRotationStkOrBtnPressed =true; //set so know what happened when this was run
                return;
            }
        }

        //At this point, stick in dead zone and buttons not pressed
        //The rotation pwr, will therefore be a corrective power to keep robot's target yaw

        //Set the target yaw when the stick or button is RELEASED after being pressed
        //From the user's stand point, robot rotates until the button/stick is released
        //and thereafter should keep the same yaw
        //NOTE:  The robot's momentum will keep it spinning slightly beyond when the user releases
        //       the stick/btn.  Will try to guess where the robot actually stopped spinning,
        //       INSTEAD of when the user released the stick/btn.
        //       The yaw at which the robot stops spinning is beyond the yaw at which the
        //       user released the stick/btn.  Will therefore try to maintain the yaw at which the
        //       robot stopped spinning.
        //
        //The momentun adjustment is throttle dependent and robot weight dependent
        //   gdMomentumAdjustmentMagDeg is therefore set when the user adjusts the throttle
        //   (since the robot weight will generally not change (unless have a HEAVY game element)
        //check if the last time entered this method, the rotation stick/btn pressed
        if(gbRotationStkOrBtnPressed) {
            //last time, the stick or button was pressed
            //this is entered the first time released, then not again until another pressed
            //set the NavX yaw PID controller to try to hold this yaw position
            if(gnRobotSpin ==CW) dMomentumCompensation= gdMomentumAdjustmentDeg;
            else if(gnRobotSpin ==CCW) dMomentumCompensation=-gdMomentumAdjustmentDeg;

            enableNavXYawPID(true);
            setpointNavXYaw(getNavXYaw()+dMomentumCompensation);
            gdRotationPwr =0;
            gnRobotSpin =NOSPIN;
            gbRotationStkOrBtnPressed =false; //so don't enter this again until then next press and release
            return;
        }

        //if got here, nothing to do but hold Yaw position
        //Rotation power is therefore a correctional power, not a power derived from the stick/btn
        gdRotationPwr = getNavXCorrectionPwr();
    }


    /**
     * Check if stick is in dead zone 
     * @param gamepadToChk driver's gamepad, typically gamepad1
     * @param bControlStickToChk Stick (STICK_RIGHT or STICK_LEFT) to check
     * @return true if stick in dead zone, false otherwise
     */
    private boolean chkStickDeadZone(Gamepad gamepadToChk, boolean bControlStickToChk) {

        //check if sticks are in dead zone
        ///CAUTION!!! Joysticks front/back are REVERSED!!!! left/right is okay-
        //     -          -
        //  -  L  +    -  R  +
        //     +          +
        if(bControlStickToChk== STICK_LEFT) {
            if (((gamepadToChk.left_stick_x <= OMNI_STICK_DEADZONE) && (gamepadToChk.left_stick_x >= -OMNI_STICK_DEADZONE)) &&
                    ((-gamepadToChk.left_stick_y <= OMNI_STICK_DEADZONE) && (-gamepadToChk.left_stick_y >= -OMNI_STICK_DEADZONE)))
                return true;
        }
        else {
            if (((gamepadToChk.right_stick_x <= OMNI_STICK_DEADZONE) && (gamepadToChk.right_stick_x >= -OMNI_STICK_DEADZONE)) &&
                    ((-gamepadToChk.right_stick_y <= OMNI_STICK_DEADZONE) && (-gamepadToChk.right_stick_y >= -OMNI_STICK_DEADZONE)))
                return true;
        }
        //out side of dead zone
        return false;
    }

/*
    private void chkResetRobotYawZero(Gamepad gamepadToChk) {
        long lCurrentTime=System.currentTimeMillis();

        //see if okay to reset yaw
        if((lCurrentTime-lBackTimestamp)<200)  return;

        if (gamepadToChk.back) { //back button hit
            if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
                //if got here, can reset yaw
                setMecWheelOff(mtrLeftBack, mtrLeftFront, mtrRightBack, mtrRightFront);
                //wait for robot to stop moving..
                navX.zeroYaw();
                //while((System.currentTimeMillis()-lCurrentTime)<300) wait();
                dYawAngleToHold = 0;
                yawPIDController.setSetpoint(dYawAngleToHold);
                bYawStkOrBtnPressed = false;
                dYawPwr = 0;
                dDirLFPwr = dDirLBPwr = dDirRFPwr = dDirRBPwr = 0;
            } else { //not using DIM.  use to control loosing gyro connection

            }

            lBackTimestamp=lCurrentTime;
        }
    }

 */

    /**
     * Turn off mecanum wheels
     * 
     * no return value.
     */
    public void setMecWheelOff() {
        gdDirectionLBPwr =0;
        gdDirectionLFPwr =0;
        gdDirectionRFPwr =0;
        gdDirectionRBPwr =0;
        gdRotationPwr =0;
        mtrLeftFront.setPower(0);
        mtrLeftBack.setPower(0);
        mtrRightFront.setPower(0);
        mtrRightBack.setPower(0);

    }

    /**
     * Set mecanum wheel directional power to 0
     * no return value.  Sets global variables gdDirectionalXXPwr to 0 XX=LF,LB,RF,RB 
     */
    private void setMecWheelDirectionPwrZero() {
        gdDirectionLFPwr = 0;  gdDirectionRFPwr = 0;
        gdDirectionLBPwr = 0;  gdDirectionRBPwr = 0;
    }

    /**
     * set the crawl power for omni control  each mecanum wheel
     * "crawling" is moving the robot in a robot oriented direction fwd, back,lefi,right
     * why would you want to do this???
     * :  Say robot is at a 57 degree angle and the game element is directly in front
     *    It is very difficult to move the direction stick at a 57 degree angle to approach the game element
     *    Instead, the driver can simply press the crawl forward button to move the robot forward
     * The crawl buttons are above the direction control stick
     *    If the gamepad left stick is the direction stick, the crawl buttons are dpad buttons
     *    If the gamepad right stick is the direction stick, the crawl buttons are A,B,X,Y
     * Note that in omni mode, the buttons above the Rotation stick already rotates the robot
     *
     * @param gamepad driver's gamepad, typically gamepad1
     * @param bDirStick LEFT_STICK for left hand operation, RIGHT_STICK for right hand operation
     * no return. global variables gdDirectionLFPwr,gdDirectionLBPwr,gdDirectionRFPwr,gdDirectionRBPwr
     *                             gbDirectionBtnPressed
     */
    private boolean setOmniCrawlPwr(Gamepad gamepad, boolean bDirStick) {
        //The crawl buttons are above the control stick
        //   If the gamepad left stick is the direction stick, the crawl buttons are dpad buttons
        //   If the gamepad right stick is the direction stick, the crawl buttons are A,B,X,Y
        boolean bOmniDirBtnPressed=false;
        if(bDirStick==STICK_RIGHT) {
            if(gamepad.y) {
                //apply pwr fwd
                gdDirectionLFPwr = gdOmniPwrMagnitude;  gdDirectionRFPwr = gdOmniPwrMagnitude;
                gdDirectionLBPwr = gdOmniPwrMagnitude;  gdDirectionRBPwr = gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_UP;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
            else if(gamepad.a) {
               //apply pwr back
                gdDirectionLFPwr = -gdOmniPwrMagnitude;  gdDirectionRFPwr = -gdOmniPwrMagnitude;
                gdDirectionLBPwr = -gdOmniPwrMagnitude;  gdDirectionRBPwr = -gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_DOWN;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
            else if(gamepad.b) {
                //move right
                gdDirectionLFPwr = gdOmniPwrMagnitude;  gdDirectionRFPwr = -gdOmniPwrMagnitude;
                gdDirectionLBPwr = -gdOmniPwrMagnitude;  gdDirectionRBPwr = gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_RIGHT;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
            else if(gamepad.x) {
                //move left
                gdDirectionLFPwr = -gdOmniPwrMagnitude;  gdDirectionRFPwr = gdOmniPwrMagnitude;
                gdDirectionLBPwr = gdOmniPwrMagnitude;  gdDirectionRBPwr = -gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_LEFT;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
        }
        if(bDirStick==STICK_LEFT) {
            if(gamepad.dpad_up) {
                //apply pwr fwd
                gdDirectionLFPwr = gdOmniPwrMagnitude;  gdDirectionRFPwr = gdOmniPwrMagnitude;
                gdDirectionLBPwr = gdOmniPwrMagnitude;  gdDirectionRBPwr = gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_UP;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            } else if(gamepad.dpad_down) {
                //apply pwr back
                gdDirectionLFPwr = -gdOmniPwrMagnitude;  gdDirectionRFPwr = -gdOmniPwrMagnitude;
                gdDirectionLBPwr = -gdOmniPwrMagnitude;  gdDirectionRBPwr = -gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_DOWN;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
            else if(gamepad.dpad_right) {
                //move right
                gdDirectionLFPwr = gdOmniPwrMagnitude;  gdDirectionRFPwr = -gdOmniPwrMagnitude;
                gdDirectionLBPwr = -gdOmniPwrMagnitude;  gdDirectionRBPwr = gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_RIGHT;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
            else if(gamepad.dpad_left) {
                //move left
                gdDirectionLFPwr = -gdOmniPwrMagnitude; gdDirectionRFPwr = gdOmniPwrMagnitude;
                gdDirectionLBPwr = gdOmniPwrMagnitude; gdDirectionRBPwr = -gdOmniPwrMagnitude;
                gnLastBtnPressed=BTN_LEFT;
                bOmniDirBtnPressed = true;
                bOmniDirBtnPressed = true;
                return bOmniDirBtnPressed;
            }
        }
        gnLastBtnPressed=BTN_NONE;
        return bOmniDirBtnPressed;
    }

     /**
      * Gets the Yaw angle the robot is trying to hold
      * @return Polar yaw angle the robot is trying to hold
      */
    public double getYawAngleToHold() {
        return gdYawAngleToHold;
    }

     /**
      * Gets the control mode of the robot
      * @return TANK_CONTROL or OMNI_CONTROL
      */
    public int getControlMode() {
        return gnControlMode;
    }

     /**
      * Get the direction stick's polar angle
      * @return Polar angle of direction stick
      */
    double getDirectionStickPolarAngle() {
        return gdDirectionStickPolar;
    }

     /**
      * Get the rotation power set for the robot at  the time this method called
      * @return Power for rotating robot
      */
    double getRotationPwr(){
        return gdRotationPwr;
    }

    /** Get the power applied to each  motor
     * The resultant power is the sum of the directional and rotational power
     * @param nMtr MTR_LF,MTR_LB,MTR_RF or MTR_RB
     * @return The power applied to nMtr at the time method is called
     */
    public double getResultantPwr(int nMtr){
        switch(nMtr) {
            case MTR_LF:
                return (gdResultantLFPwr);
            case MTR_RF:
                return (gdResultantRFPwr);
            case MTR_LB:
                return (gdResultantLBPwr);
            case MTR_RB:
                return (gdResultantRBPwr);
        }
        return 0d;
    }

    /**
     * Get type of tank movement 
     * @return  tank movement in operation
     */
    public String getTankMovement() {
        switch (gnTankMovement) {
        case TANK_NO_MOVEMENT:
            return  "No Tank Movement";
        case TANK_VERTICAL_MOVEMENT:
            return  "Vertical Tank Movement";
        case TANK_HORIZONTAL_MOVEMENT:
            return  "Horizontal Tank Movement";
        case TANK_SPIN_MOVEMENT:
            return  "Spin Tank Movement";
        case TANK_TURN_MOVEMENT:
            return  "Turn Tank Movement";
        default:
            return "Unknown Tank Movement";

        }
    }

    /**
     * Get string indicating the navX status
     * @return status of NavX
     */
    public String getNavXStatus() {
        switch(gnNavXStatus) {
            case NAVX_ERROR:
                return "NavX Error";
            case NAVX_KAUAILABS_LIB:
                return ("NavX using KL Lib");
            case NAVX_QC_LIB:
                return ("NavX using QC Lib");
            case NAVX_NONE:
                return ("No NavX");
        }
        return "No NavX Status";
    }

     /**
      * Get int indicating the navX status
      * @return status of NavX NAVX_ERROR,NAVX_KAUAILABS_LIB,NAVX_QC_LIB,NAVX_QC_NONE
      */
     public int  getNavXStat() {
         return gnNavXStatus;
     }


    /**
     * Check if navX is connected
     *
     * @return NavXConnection stauts
     */

    public boolean isNavXConnected() {
        //SkyStone v5.2 incompatibility
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            return (navX.isConnected());
        } else {
            //no equivalent without dim, so send true an pray
            return true;
        }
    }

    /**
     * Get the yaw angle from the NavX
     * 
     * @return  Yaw angle of the robot
     */
    public double getNavXYaw() {
        //SkyStone v5.2 incompatibility
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if(navX.isConnected())
                return (double) (navX.getYaw());
            else return 0;
        } else if(gnNavXStatus == NAVX_QC_LIB) {
            try {
                AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
                Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gnNavXStatus= NAVX_QC_LIB;
                return (double) (-(angles.firstAngle)); //Darn FIRST implementation returns a neg angle
            } catch (Exception e) { //this does not catch condition of a failed navx.
                //Thread.currentThread().interrupt();
                gnNavXStatus=NAVX_ERROR;
                return 0;
            }
        } else {
            return (double) 0;
        }
    }

     /**
      * Close navX
      */
    public void closeNavX()  {
        //No SkyStone v5.2 incompatibility
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if(yawPIDController.isEnabled()) yawPIDController.enable(false);
            yawPIDController.close();
            navX.close();
        }
        else {
            //nothing equivalent in gyro classes
        }
    }

     /**enable the navX
      *  Only change state if necessary
      * @param bEnable
      */
    private void enableNavXYawPID(boolean bEnable)  {
        /*
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if(bEnable==true) { //want to enable the yawPIDController
                if(!yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
        }
         */
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if(bEnable==true) { //want to enable the yawPIDController
                if(!yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
            else { //want to disable yawPIDController
                if(yawPIDController.isEnabled()) yawPIDController.enable(bEnable);
            }
        }
        //no equivalent in FIRST library
    }

    /**
     *Set the robot orientation (yaw angle) to hold
     * The robot will try to hold this angle
     * The angle to hold is a polor angle. The 0 angle is where the navX calibration was made
     * in initNavX
     * @param dAngleToSet the polar angle to hold -180-0-180
     */
    private void setpointNavXYaw(double dAngleToSet) {
        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
           yawPIDController.setSetpoint(dAngleToSet);
        }
        else {
            setCrudeCorrectionSetPoint(dAngleToSet);
        }
        //must set dYawAngleToHold regardless
        gdYawAngleToHold =dAngleToSet;
    }

    /**
     * Get the power magnitude  to apply to each wheel to rotate the robot to the previously set
     * global variable gdYawAngleToHold
     * @return Power, -1 to 1, to correct the robot's current yaw angle to gdYawAngleToHold
     */
    public double getNavXCorrectionPwr() {
        //prior to calling this, the user call setpointNavXYaw to set the target angle
        //   the difference between the robot's current angle and the target angle
        //   determines the magnitude and sign of the output returned.
        //SkyStone v5.2 incompatibility

        if(gnNavXStatus==NAVX_KAUAILABS_LIB) {
            if (navX.isConnected()) {
                navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
                try {
                    if (yawPIDController.waitForNewUpdate(yawPIDResult,NAVX_TIMEOUT)) {
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
        } else if (gnNavXStatus == NAVX_QC_LIB){
            return calcCrudeYawCorrectionPwr();
        } else {
            return 0;
        }
    }


     /**
     * Crude way of calculating the rotate to angle correction power
     *
     * The returned value is meant to add to the left wheels (and subtract from the right wheels)
     *
     *uses global variable  glCrudeYawCorrectionLast;
     * @return the correction power between -1d and 1d
     */
    private double calcCrudeYawCorrectionPwr() {
        double dTargetPolar=gdYawAngleToHold,dCurrentPolar=getNavXYaw();
        //To tune below
        //create auton program to rotate 90 deg
        //turn I off Yawpido=0
        //adust ypidp to get to close to angle without passing it
        //adust ypidi to get the rest of the way to 90

        double CCYAW_P = 4d*(.75d/180d);  //180 correction uses .75 power; dist to rotate will be <=180
        double CCYAW_I = .0001d;
        long CCYAW_TIMEOUT=15000; //5 seconds
        double dCalc;
        long lCurrentTime=System.currentTimeMillis();
        double dDistToRotate;

        if((lCurrentTime-glCrudeYawTimeLast)>CCYAW_TIMEOUT) {
            glCrudeYawTimeLast =lCurrentTime;
            gdCrudeIPwr=0;
            gdCrudePPwr=0;
            gdCrudeYawDistLast =0d;
            gnCrudeYawSpin =NOSPIN;
            return 0d;
        }
        //find out how much need to correct, and in what direction
        dDistToRotate=calcDistToRotateDegrees(dTargetPolar,dCurrentPolar);

        //At this point, have the shortest degrees to rotate, and the direction
        //can now calculate the yaw correctional power
        //check if within tolerance.. dDegToRotate is a magnitude, TOLERANCE is a magnitude
        if(Math.abs(dDistToRotate)<TOLERANCE_DEGREES) {
            //within limits
            glCrudeYawTimeLast =lCurrentTime;
            gdCrudeIPwr=0;
            gdCrudePPwr=0;
            gdCrudeYawDistLast =0d;
            gnCrudeYawSpin =NOSPIN;
            return 0d;
        }

        //Calculate the P, or proportional power.  The larger the degress to rotate,
        //   the larger the P
        gdCrudePPwr=CCYAW_P*dDistToRotate;

        //Calcuolate the I
        //see how much the Yaw has changed since the last time
        //set I depending on how much change occured.
        //if little change has occurred, want to increase I
        //
        if(gnCrudeYawSpin==CCW) { //robot spin is CCW
            //if got here, know that the correction Dist initially set was neg or CCW
            //check if the current correction distance is pos or neg
            if(dDistToRotate<0) {
                //the distance to currently rotate is also neg
                //can compare the current distance to rotate and the last distance to rotate directly
                if(gdCrudeYawDistLast<dDistToRotate) {
                    // -180---------------0
                    //          | Current |
                    //      !    last     |
                    //
                    //if got here the distance to rotate is getting smaller
                    //correction power should be getting smaller
                    //going in CCW direction and correction is CCW
                    //    so need to reduce magnitude, but keep same direction
                    dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                    gdCrudeIPwr-=dCalc; //reduce correction by adding since initial spin is CCW
                } else {
                    // -180---------------0
                    //     |   Current    |
                    //          !  last   |
                    //
                    //if got here the distance to rotate is getting larger or not changed,
                    // so need to apply more power
                    //going in CCW direction and correction is CCW
                    //    so need to increase magnitude, but keep same direction
                    dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                    gdCrudeIPwr+=dCalc; //increase correction subtracting since initial spin is CW
                }
            } else if (dDistToRotate>0) {
                //if got here, know that the correction initially set was neg or CCW
                //the distance to currently rotate is positive
                //this means that have an overshoot
                //calc magnitude of correction and apply in opposite direction of initial spin
                dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                gdCrudeIPwr-=dCalc; //correction by adding
            }
        } else if(gnCrudeYawSpin==CW) { //robot spin is CCW
            //if got here, know that the correction Dist initially set was neg or CCW
            //check if the current correction distance is pos or neg
            if(dDistToRotate>0) {
                //the distance to currently rotate is also neg
                //can compare the current distance to rotate and the last distance to rotate directly
                if(gdCrudeYawDistLast>dDistToRotate) {
                    // -0---------------180
                    // | Current |
                    // !    last     |
                    //
                    //if got here the distance to rotate is getting smaller
                    //correction power should be getting smaller
                    //going in CCW direction and correction is CCW
                    //    so need to reduce magnitude, but keep same direction
                    dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                    gdCrudeIPwr+=dCalc; //reduce correction by adding since initial spin is CCW
                } else {
                    // -180---------------0
                    //     |   Current    |
                    //          !  last   |
                    //
                    //if got here the distance to rotate is getting larger or not changed,
                    // so need to apply more power
                    //going in CCW direction and correction is CCW
                    //    so need to increase magnitude, but keep same direction
                    dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                    gdCrudeIPwr-=dCalc; //increase correction subtracting since initial spin is CW
                }
            } else if (dDistToRotate<0) {
                //if got here, know that the correction initially set was neg or CCW
                //the distance to currently rotate is positive
                //this means that have an overshoot
                //calc magnitude of correction and apply in opposite direction of initial spin
                dCalc=CCYAW_I*((double)lCurrentTime-(double)glCrudeYawTimeLast); //magnitude of correction
                gdCrudeIPwr+=dCalc; //correction by adding
            }
        } else { //no inital spin
           gdCrudePPwr=0;
           gdCrudeIPwr=0;
        }


        //add P + I , give direction, then return
        glCrudeYawTimeLast =lCurrentTime;
        gdCrudeYawDistLast=dDistToRotate;

        dCalc=gdCrudePPwr;
        dCalc+=gdCrudeIPwr;
        return dCalc;
    }

    private void setCrudeCorrectionSetPoint(double dPolarAngle) {
        double dTarget;
        double dCurrent=getNavXYaw();

        dTarget=fixPolar(dPolarAngle);
        gdCrudeYawDistLast =calcDistToRotateDegrees(dTarget,dCurrent);
        if(gdCrudeYawDistLast <0) gnCrudeYawSpin =CCW;
        else if(gdCrudeYawDistLast >0) gnCrudeYawSpin = CW;
        else  gnCrudeYawSpin = NOSPIN;
        glCrudeYawTimeLast =System.currentTimeMillis();
        gdCrudeIPwr= gdCrudePPwr =0d;

    }


     /**
      * Get the I power from the CrudeYawCorrection
      * @return gdCrudeIPwr
      */
     public double getCrudeIPwr() {
         return gdCrudeIPwr;
     }

     /**
      * Get the P power from the CrudeYawCorrection
      * @return gdCrudePPwr
      */
     public double getCrudePPwr() {
         return gdCrudePPwr;
     }

     /**
      * Calculate the smallest distance between the target and current robot rotation angle
      * @param dTargetPolar Polar angle to target
      * @param dCurrentPolar Current polar angle
      * @return magnitude of the smallest distance between target and current polar angle. Positive if need to rotate clockwise to travel smallest distance, negative if need to rotate counterclockwise
      */
    private double calcDistToRotateDegrees (double dTargetPolar, double dCurrentPolar) {
        double  dDegToRotate=0;
        int nRotationDirection=0;
        double dTstCCW, dTstCW;

        dCurrentPolar=fixPolar(dCurrentPolar);
        dTargetPolar=fixPolar(dTargetPolar);
        //first, find the distance need to rotate for correction in polar degree
        //   calculate the distance to rotate CW
        //   calculate the distance to rotate CCW,
        //   use the smaller distance
        //
        //Sample calculations
        //    Current    Target    Correction           Dir
        //    --------   ------    -----------------
        //    0 180    >  0 180    abs(target-current)  CCW
        //    0 180    <  0 180    abs(target-current)   CW
        //    0 180      -180 0    current+abs(target)  CCW
        //                         (180-current) + abs(-180-target)  CW
        //   -180 0    > -180 0    abx(target-current)  CCW
        //   -180 0    < -180 0    abx(target-current)  CW
        //   -180 0       0 180    abs(current)+target   CW
        //                0 180    (-180-current)+ (180-target) CCW
        if((dCurrentPolar>=0)&&(dCurrentPolar<=180)) {
            if((dTargetPolar>=0)&&(dTargetPolar<=180)) {
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //    0 180    >  0 180    abs(target-current)  CCW
                //    0 180    <  0 180    abs(target-current)   CW
                if (dTargetPolar <= dCurrentPolar) {
                    //         |  T
                    //         |    C
                    //         |
                    // - - - - + - - - -
                    //if here need to move counter clockwise
                    dDegToRotate = dCurrentPolar-dTargetPolar; //make dDegToRotate pos
                    if(dDegToRotate==0) nRotationDirection=0;
                    else nRotationDirection = -1; //Counter clockwise movement
                } else {  //dTargetPolar>dCurrentPolar
                    //         |  C
                    //         |    T
                    //         |
                    // - - - - + - - - -
                    //         |
                    //         |    C
                    //         |  T
                    //if here need to move clockwise
                    dDegToRotate = dTargetPolar-dCurrentPolar; //make dDegToRotate pos
                    nRotationDirection = 1; //Clockwise movement
                }
            //at this point, recall that this is still the condition: dCurrentPolar>=0 && dCurrentPolar<=180
            } else { //dTargetPolar<0 && dTargetPolar <= -180
                //find shortest path
                //         |  C1
                //     T1  |   C3
                //    T2   |
                // - - - - + - - - -
                //    T3   |   C2
                //     T4  |  C4
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //    0 180      -180 0    current+abs(target)  CCW
                //                         (180-current) + abs(-180-target)  CW
                //Check dist for a CCW turn
                dTstCCW=dCurrentPolar+Math.abs(dTargetPolar);
                //Check dist for a CC turn
                dTstCW=(180d-dCurrentPolar)+(180d+dTargetPolar);
                if(dTstCCW>=dTstCW) {
                    //CW is shorter path
                    dDegToRotate=dTstCW;
                    if(dDegToRotate==0) nRotationDirection=0;
                    nRotationDirection = 1; //Clockwise movement
                } else { //dTstCCW<dTstCW
                    //CCW is shortest path
                    dDegToRotate=dTstCCW;
                    nRotationDirection = -1; //Counterclockwise
                }

            }
        } else  {
            //((dCurrentPolar<0)&&(dCurrentPolar>=-180)) {
            //perhaps can combine with above conditional
            //but.. do this way to help debugging
            if((dTargetPolar<0)&&(dTargetPolar>=-180)) {
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //   -180 0    > -180 0    abx(target-current)  CCW
                //   -180 0    < -180 0    abx(target-current)  CW
                //At a negative current  polar angle
                if (dTargetPolar <= dCurrentPolar) { //target is also negative
                    //      C  |
                    //    T    |
                    // - - - - + - - - -
                    //         |
                    //    C    |
                    //      T  |
                    //if here need to move counterclockwise
                    dDegToRotate = dCurrentPolar-dTargetPolar; //make dDegToRotate pos
                    if(dDegToRotate==0) nRotationDirection=0;
                    else nRotationDirection = -1; //counterclockwise
                } else { //dTargetPolar>dCurrentPolar)
                    //      T  |
                    //    C    |
                    //         |
                    // - - - - + - - - -
                    //         |
                    //    T    |
                    //      C  |
                    //if here need to move clockwise
                    dDegToRotate = dTargetPolar - dCurrentPolar; //make dDegToRotate pos
                    nRotationDirection = 1; //Clockwise movement
                }
            //((dCurrentPolar<0)&&(dCurrentPolar>=-180)) {
            } else { //dTargetPolar >= 0 && dTargetPolar<=180
                //find shortest path
                //         | T
                //     C   |
                //         |
                // - - - - + - - - -
                //         |       T
                //         |
                //    Current    Target    Correction           Dir
                //    --------   ------    -----------------
                //   -180 0       0 180    abs(current)+target   CW
                //                0 180    current-(-180) + 180 - target  CCW
                //
                //   Ex Current = -8  Target =0  CW = abs(-8)+0= 8  CCW=(-8)+180 + 180 = 352

                dTstCW=Math.abs(dCurrentPolar)+dTargetPolar;
                dTstCCW=(dCurrentPolar+180d) + (180d-dTargetPolar);
                if(dTstCCW>=dTstCW) {
                     //CW is shorter path
                    dDegToRotate=dTstCW;
                    if(dDegToRotate==0) nRotationDirection=0;
                    else nRotationDirection = 1; //Clockwise movement
                } else {
                    dDegToRotate=dTstCCW;
                    nRotationDirection = -1; //Counterclockwise
                }
            }
        }
        return dDegToRotate*(double)nRotationDirection;
    }

    /**
     * Ensure valid polar angle
     * @param dPolarToFix  The polar angle to check
     * @return  The valid polar angle
     */
    private double fixPolar(double dPolarToFix) {

        double dRes=dPolarToFix;

        dPolarToFix %= 360.0; //normalize  to 0 to 360;

        if(dPolarToFix>(double)180) {
            //more than 180, so polar is actually negative
            dRes=dPolarToFix-(double)360;
        } else if(dPolarToFix<-(double)180)  {
            dRes=dPolarToFix+(double)360;
        }
        return dRes;
    }

    private boolean initNavXKL(OpMode opMode,String strNavXName) {
        //instantiating class auto starts navigation
        boolean bCalibrationComplete = false;
        long lStartTime=System.currentTimeMillis();
        
        navX = getInstance(opMode.hardwareMap.get(NavxMicroNavigationSensor.class, strNavXName),
                DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);

        opMode.telemetry.addData("navX-Micro", "Startup Calibration");
        opMode.telemetry.update();
        while ((!bCalibrationComplete)&&((System.currentTimeMillis()-lStartTime)<30000)) {
            // navX-Micro Calibration completes automatically 15 seconds after it is
            //powered on, as long as the device is still.  To handle the case where the
            //navX-Micro has not been able to calibrate successfully, hold off using
            //the navX-Micro Yaw value until calibration is complete.

            bCalibrationComplete = !navX.isCalibrating();
            //opMode.telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            //opMode.telemetry.update();
            //opMode.wait(50);
            //opMode.idle();
        }
        opMode.telemetry.addData("navX-Micro", "calibration %s",bCalibrationComplete?"complete":"error");
        opMode.telemetry.update();
        if(!bCalibrationComplete) return false;
        navX.zeroYaw();
        //Create a PID Controller which uses the Yaw Angle as input.
        yawPIDController = new navXPIDController(navX,
                navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        return true;
    }


    /**
     * Calibrate navX using QualComm libraries
     * @param opMode opmode calling this method
     * @param strNavXName name of navX in robot configuration
     * @return  NAVX_QC_LIB navX using QualComm Libraries, NAVX_ERROR problem configuring navX, NVX_NONE cannot find navX,
     * global variable gyro set
     */
    private int initNavXQC(OpMode opMode,String strNavXName) {
        NavxMicroNavigationSensor navxMicro;
        try {
            long lTimeStart=System.currentTimeMillis();
            boolean bTimeout=false;

            navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, strNavXName);
            gyro = (IntegratingGyroscope) navxMicro;

            // The gyro automatically starts calibrating. This takes a few seconds.
            opMode.telemetry.addData("Gyro:", "Calibrating. Do Not Move!");
            opMode.telemetry.update();

            // Wait until the gyro calibration is complete
            while (navxMicro.isCalibrating()&&!bTimeout) {
                opMode.telemetry.addData("calibrating", "...");
                opMode.telemetry.update();
                if((System.currentTimeMillis()-lTimeStart)<10000) bTimeout=true; //10 second timeout
                Thread.sleep(50);
            }
            if(bTimeout) return  NAVX_ERROR;
            return NAVX_QC_LIB;
        }
        catch (IllegalArgumentException e) {
            opMode.telemetry.addData("argument", "%s", e.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return NAVX_NONE;
        }
        catch (InterruptedException e){
            opMode.telemetry.addData("interrupt", "%s", e.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        }
        catch (InternalError e) {
            opMode.telemetry.addData("internal error", "%s", e.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        }
        catch (Exception e) {
            opMode.telemetry.addData("internal error", "%s", e.toString());
            opMode.telemetry.update();
            //Thread.currentThread().interrupt();
            return NAVX_ERROR;
        }
    }

     /**
      * Reset the motors for teleop
      * @param bUseEncoders true=use encoders, false= don't use encoders
      */
    public void resetChassisTeleop(boolean bUseEncoders) {
        /*4 motors, so must sync so use encoders, do NOT stop and reset or will not run*/
        if(bUseEncoders) setMotorsWithEncoders(true);
        else setMotorsWithoutEncoders();
        mtrRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     /**
      * Reset the motors for autonomous
      */
    public void resetChassisAuton(boolean bUseEncoders) {
        if(bUseEncoders) setMotorsWithEncoders(true);
        else setMotorsWithoutEncoders();
        mtrLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     /**
      * Invoking this method this with 'true' frees the left trigger and bummper
      * One possible use:  This allows another class, like an input control class
      * to put the input controls on the driver's left trigger and bumper.
      * Setting this back to false puts the incremental speed controls on
      * the left trigger and bumper
      * The right bumper is high speed, the right trigger is low speed
      * @param bDontUse
      */
    public void chassisDontUseLeftTrigBmpr(boolean bDontUse) {
            gbDontUseLeftTrigBmpr = bDontUse;
    }

     /**
      * Setup motors with encoders
      * @param bReset true=STOP_AND_RESET_ENCODER
      */
    private void setMotorsWithEncoders(boolean bReset) {
        mtrLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(bReset) mtrLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mtrRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (bReset) mtrRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mtrRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(bReset) mtrRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        mtrLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(bReset) mtrLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

     /**
      * Setup motors without encoders
      */
    private void setMotorsWithoutEncoders() {
        mtrLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

     /**
      * Get power setting used in omni mode at the time method is called
      * @return Omni power, -1 to 1 used in OMNI_CONTROL
      */
    public double getOmniPwr() {
        return gdOmniPwrMagnitude;
    }

     /**
      * Get the momentum adjustment used for rotating robot in omni mode
      * @return momentum adjustment in degrees
      */
    public double getMomentumAdj() {
        return gdMomentumAdjustmentDeg;
    }

     /**
      * Get power setting used in tank mode at the time method is called
      * @return Magnitude of the Tank power, -1 to 1 used in TANK_CONTROL
      */
    public double getTankPwrMagnitude() {
        return(gdTankPwrMagnitude);
    }

    /**
     * gets DcMotor's getCurrentPosition()
     * @param nMtr motor number to return
     * @return DCMotor fot eh
     */
    public int getMtrCurrentPosition(int nMtr) {
        switch(nMtr) {
            case MTR_RF:
                return mtrRightFront.getCurrentPosition();
            case MTR_LB:
                return mtrLeftBack.getCurrentPosition();
            case MTR_RB:
                return mtrRightBack.getCurrentPosition();
            case MTR_LF:
            default:
                return mtrLeftFront.getCurrentPosition();
        }
    }

    /**
     * gets DcMotor
     * Use with caution if you change motor parameters.
     * @param nMtr number to return
     * @return DCMotor fot eh
     */
    public DcMotor getDcMotor(int nMtr) {
        switch(nMtr) {
            case MTR_RF:
                return mtrRightFront;
            case MTR_LB:
                return mtrLeftBack;
            case MTR_RB:
                return mtrRightBack;
            case MTR_LF:
            default:
                return mtrLeftFront;
        }
    }



     /**
      * Calculate the
       * @return the maximum directional power applied to any wheel
      */
    private double getMaxDirPwr() {
        double dMaxDirPwr=0;
        if(Math.abs(gdDirectionLFPwr) > dMaxDirPwr) dMaxDirPwr=Math.abs(gdDirectionLFPwr);
        if(Math.abs(gdDirectionLBPwr) > dMaxDirPwr) dMaxDirPwr=Math.abs(gdDirectionLBPwr);
        if(Math.abs(gdDirectionRFPwr) > dMaxDirPwr) dMaxDirPwr=Math.abs(gdDirectionRFPwr);
        if(Math.abs(gdDirectionRBPwr) > dMaxDirPwr) dMaxDirPwr=Math.abs(gdDirectionRBPwr);
       return dMaxDirPwr;
    }


    //Warning!! must call the following in Autonomous LinearOpMode
    public int autonpivotTurnNavX(LinearOpMode opMode,double dTargetHeading,long lMaxTime_msec) {
        long lTimeStamp=System.currentTimeMillis();

        //correct input
        if(dTargetHeading>=0) { //positive target heading
            if(dTargetHeading>360) dTargetHeading%=360; //make target less than 360
            if(dTargetHeading>180) {//target is too positive, make it negative
                //target is between 180 and 360o
                dTargetHeading = dTargetHeading - 360;
            }
        }
        else  { //negative target heading
            if(dTargetHeading<-360) dTargetHeading%=360; //make target less than -360
            if(dTargetHeading<-180) {//target is too negative, make it positive
                dTargetHeading = dTargetHeading + 360;
            }
        }

        setpointNavXYaw(dTargetHeading);
        while (opMode.opModeIsActive()&&!opMode.isStopRequested()&&
                !Thread.currentThread().isInterrupted()) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime_msec) {
                setMecWheelOff();
                return TIMEDOUT;
            }

            gdRotationPwr = getNavXCorrectionPwr();

            mtrLeftBack.setPower(gdRotationPwr);
            mtrLeftFront.setPower(gdRotationPwr);
            mtrRightBack.setPower(-gdRotationPwr);
            mtrRightFront.setPower(-gdRotationPwr);

            opMode.telemetry.addData("Direction","Target:%.2f Current %.2f",dTargetHeading,getNavXYaw());
            opMode.telemetry.addData("Yaw Pwr","%.2f P:%.2f I:%.2f dist:%.2f", gdRotationPwr,gdCrudePPwr,gdCrudeIPwr,gdCrudeYawDistLast);
            opMode.telemetry.update();

            if((gdRotationPwr<.001)&&(gdRotationPwr>-.001)) { //so dont' have to do a double == compare
                setMecWheelOff();
                return 0;
            }

        }

        //if got here, or something bad happened
        setMecWheelOff();
        return INTERRUPTED;
    }


    public int autonmoveNavX(LinearOpMode opMode,int nDistToMoveEncoderCount, long lMaxTime,
                                         double dRobotOrientationToMaintain, int nDir,double dPwr) {

        long lTimeStamp = System.currentTimeMillis();
        int nStartCount = mtrLeftFront.getCurrentPosition();
        int nTargetCount=0;
        double dPwrCalc=Math.abs(dPwr);

        int nDistanceRemaining=0, nDistanceTravelled=0;

        switch(nDir) {
            case AUTON_FWD:

            case AUTON_RIGHT:
                nTargetCount= nStartCount + nDistToMoveEncoderCount;//since moving wheel fwd or right
                break;
            case AUTON_BACK:
            case AUTON_LEFT:
                nTargetCount = nStartCount - nDistToMoveEncoderCount;//since moving wheel backwards
                break;
        }

        setpointNavXYaw(dRobotOrientationToMaintain);

        switch(nDir) {
            case AUTON_FWD:
                gdDirectionLBPwr =dPwrCalc;
                gdDirectionLFPwr =dPwrCalc;
                gdDirectionRFPwr =dPwrCalc;
                gdDirectionRBPwr =dPwrCalc;
                break;
            case AUTON_BACK:
                gdDirectionLBPwr =-dPwrCalc;
                gdDirectionLFPwr =-dPwrCalc;
                gdDirectionRFPwr =-dPwrCalc;
                gdDirectionRBPwr =-dPwrCalc;
                break;
            case AUTON_LEFT:
                gdDirectionLFPwr =-dPwrCalc; gdDirectionRFPwr = dPwrCalc;
                gdDirectionLBPwr = dPwrCalc; gdDirectionRBPwr =-dPwrCalc;
                break;
            case AUTON_RIGHT:
                gdDirectionLFPwr = dPwrCalc; gdDirectionRFPwr =-dPwrCalc;
                gdDirectionLBPwr =-dPwrCalc; gdDirectionRBPwr = dPwrCalc;
                break;
        }
        while (opMode.opModeIsActive()&& !opMode.isStopRequested() && !Thread.currentThread().isInterrupted()) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                setMecWheelOff();
                return TIMEDOUT;
            }
            switch(nDir) {
                case AUTON_FWD:
                case AUTON_RIGHT:
                    //since moving wheel fwd or right
                    nDistanceRemaining = nTargetCount - mtrLeftFront.getCurrentPosition();
                    nDistanceTravelled = mtrLeftFront.getCurrentPosition() - nStartCount;
                    break;
                case AUTON_BACK:
                case AUTON_LEFT:
                    //since moving wheel back or left
                    // target=-100 start =-10 current =-20
                    // target=50  start=100 current=90
                    // target=-20  start =50 current=30 dist=30-(-20)=50
                    // target=-20  start =50 current=-10 dist=-10-(-20)=10
                    nDistanceRemaining = mtrLeftFront.getCurrentPosition()-nTargetCount;
                    // start =50 current=30 dist=50-30=20
                    // target=-20  start =50 current=-10 dist=50-(-10)=60
                    nDistanceTravelled = nStartCount-mtrLeftFront.getCurrentPosition();
                    break;
            }

            switch(nDir) {
                case AUTON_FWD:
                case AUTON_RIGHT:
                    if (mtrLeftFront.getCurrentPosition() >= nTargetCount) {
                        setMecWheelOff();
                        return (mtrLeftFront.getCurrentPosition()-nStartCount);
                    }
                    break;
                case AUTON_BACK:
                case AUTON_LEFT:
                    if (mtrLeftFront.getCurrentPosition() <= nTargetCount) {
                        setMecWheelOff();
                        return (nStartCount-mtrLeftFront.getCurrentPosition());
                    }
                    break;
            }

            gdRotationPwr = getNavXCorrectionPwr();

            addDirectionAndRotationPwr();
            mtrLeftFront.setPower(gdResultantLFPwr);
            mtrRightFront.setPower(gdResultantRFPwr);
            mtrLeftBack.setPower(gdResultantLBPwr);
            mtrRightBack.setPower(gdResultantRBPwr);
            opMode.telemetry.addData("Direction", "Orientation:%.2f Current %.2f",
                    dRobotOrientationToMaintain, getNavXYaw());
            opMode.telemetry.addData("Pwr", "LF %.2f RF %.2f", gdResultantLFPwr, gdResultantRFPwr);
            opMode.telemetry.addData("   ", "LB %.2f RB %.2f", gdResultantLBPwr, gdResultantRBPwr);
            opMode.telemetry.update();

        }
        //if got here, something bad happened
        setMecWheelOff();
        return INTERRUPTED;
    }

    private void autonsetControlledDirPwr(int nDistanceTraveled, int nDistanceRemaining, int nDir) {
        final double MIN_PWR = .25, MAX_PWR=1.0;
        final int ACCEL_DIST = 1000;
        final int DECEL_DIST = 1500;

        double dPwrCalc=1.0; //full power when not accelerating or decelerating

        //accelerate
        if(nDistanceTraveled<ACCEL_DIST) {
            //linear
            // y=mx+b
            // y=pwr  x=dist m=slope=MAX_PWR/ACCEL_DIST
            //
            //   |       *
            //   |     *
            //   |   *
            //   | *
            // - + - - - | - - -
            //           ACCEL_DIST
            //
            dPwrCalc=(MAX_PWR/(double)ACCEL_DIST)*(double)nDistanceTraveled;
            //  y   m         x
            // .5  (1/1000)   500
            // .1  (1/1000)   100
            if(dPwrCalc<MIN_PWR) dPwrCalc=MIN_PWR;
        }

        //decelerate
        if(nDistanceRemaining<DECEL_DIST) {
            //linear
            // y=-mx+b
            // y=pwr  x=dist m=slope=MAX_PWR/DECEL_DIST
            //
            //   |       *
            //   |     *
            //   |   *
            //   | *
            // - + - - - | - - -
            //           DECEL_DIST
            //
            dPwrCalc=(MAX_PWR/(double)ACCEL_DIST)*(double)nDistanceTraveled;
            //  y   m         x
            // .5  (1/1500)   750
            // .25  (1/1500)   375
            if(dPwrCalc<MIN_PWR) dPwrCalc=MIN_PWR;
        }
        switch(nDir) {
            case AUTON_FWD:
                gdDirectionLBPwr =dPwrCalc;
                gdDirectionLFPwr =dPwrCalc;
                gdDirectionRFPwr =dPwrCalc;
                gdDirectionRBPwr =dPwrCalc;
                break;
            case AUTON_BACK:
                gdDirectionLBPwr =-dPwrCalc;
                gdDirectionLFPwr =-dPwrCalc;
                gdDirectionRFPwr =-dPwrCalc;
                gdDirectionRBPwr =-dPwrCalc;
                break;
            case AUTON_LEFT:
                gdDirectionLFPwr =-dPwrCalc; gdDirectionRFPwr = dPwrCalc;
                gdDirectionLBPwr = dPwrCalc; gdDirectionRBPwr =-dPwrCalc;
                break;
            case AUTON_RIGHT:
                gdDirectionLFPwr = dPwrCalc; gdDirectionRFPwr =-dPwrCalc;
                gdDirectionLBPwr =-dPwrCalc; gdDirectionRBPwr = dPwrCalc;
                break;
        }

    }
    public int autonmoveControlledNavX(LinearOpMode opMode,int nDistToMoveEncoderCount, long lMaxTime,
                                      double dRobotOrientationToMaintain, int nDir) {

        long lTimeStamp = System.currentTimeMillis();
        int nStartCount = mtrLeftFront.getCurrentPosition();
        int nTargetCount=0;

        int nDistanceRemaining=0, nDistanceTravelled=0;

        switch(nDir) {
            case AUTON_FWD:
            case AUTON_RIGHT:
                nTargetCount= nStartCount + nDistToMoveEncoderCount;//since moving wheel fwd or right
                break;
            case AUTON_BACK:
            case AUTON_LEFT:
                nTargetCount = nStartCount - nDistToMoveEncoderCount;//since moving wheel backwards
                break;
        }

        setpointNavXYaw(dRobotOrientationToMaintain);

        while (opMode.opModeIsActive()&& !opMode.isStopRequested() && !Thread.currentThread().isInterrupted() ) {
            if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                setMecWheelOff();
                return TIMEDOUT;
            }
            switch(nDir) {
                case AUTON_FWD:
                case AUTON_RIGHT:
                    //since moving wheel fwd or right
                    nDistanceRemaining = nTargetCount - mtrLeftFront.getCurrentPosition();
                    nDistanceTravelled = mtrLeftFront.getCurrentPosition() - nStartCount;
                    break;
                case AUTON_BACK:
                case AUTON_LEFT:
                    //since moving wheel back or left
                    // target=-100 start =-10 current =-20
                    // target=50  start=100 current=90
                    // target=-20  start =50 current=30 dist=30-(-20)=50
                    // target=-20  start =50 current=-10 dist=-10-(-20)=10
                    nDistanceRemaining = mtrLeftFront.getCurrentPosition()-nTargetCount;
                    // start =50 current=30 dist=50-30=20
                    // target=-20  start =50 current=-10 dist=50-(-10)=60
                    nDistanceTravelled = nStartCount-mtrLeftFront.getCurrentPosition();
                    break;
            }

            autonsetControlledDirPwr(nDistanceTravelled, nDistanceRemaining, nDir);
            switch(nDir) {
                case AUTON_FWD:
                case AUTON_RIGHT:
                    if (mtrLeftFront.getCurrentPosition() >= nTargetCount) {
                        setMecWheelOff();
                        return (mtrLeftFront.getCurrentPosition()-nStartCount);
                    }
                    break;
                case AUTON_BACK:
                case AUTON_LEFT:
                    if (mtrLeftFront.getCurrentPosition() <= nTargetCount) {
                        setMecWheelOff();
                        return (nStartCount-mtrLeftFront.getCurrentPosition());
                    }
                    break;
            }

            gdRotationPwr = getNavXCorrectionPwr();

            addDirectionAndRotationPwr();

            if (nDir==AUTON_RIGHT) { //if the right button was pressed
                mtrRightBack.setPower(gdResultantRBPwr);
                mtrLeftBack.setPower(gdResultantLBPwr);
                mtrLeftFront.setPower(gdResultantLFPwr);
                mtrRightFront.setPower(gdResultantRFPwr);
            } else if (nDir==AUTON_LEFT) { //if the left button was pressed
                mtrRightBack.setPower(gdResultantRBPwr);
                mtrLeftBack.setPower(gdResultantLBPwr);
                mtrLeftFront.setPower(gdResultantLFPwr);
                mtrRightFront.setPower(gdResultantRFPwr);
            } else { //up and down buttons
                mtrRightBack.setPower(gdResultantRBPwr);
                mtrLeftBack.setPower(gdResultantLBPwr);
                mtrLeftFront.setPower(gdResultantLFPwr);
                mtrRightFront.setPower(gdResultantRFPwr);
            }
            opMode.telemetry.addData("Direction", "Orientation:%.2f Current %.2f",
                    dRobotOrientationToMaintain, getNavXYaw());
            opMode.telemetry.addData("Pwr", "LF %.2f RF %.2f", gdResultantLFPwr, gdResultantRFPwr);
            opMode.telemetry.addData("   ", "LB %.2f RB %.2f", gdResultantLBPwr, gdResultantRBPwr);
            opMode.telemetry.update();

        }
        //if got here, interrupted
        setMecWheelOff();
        return INTERRUPTED;
    }

     public int autonmoveNavXToLine(LinearOpMode opMode,int nDistToMoveEncoderCount, long lMaxTime,
                              double dRobotOrientationToMaintain, int nDir,double dPwr,double dAlphaThreshPct) {

         long lTimeStamp = System.currentTimeMillis();
         int nStartCount = mtrLeftFront.getCurrentPosition();
         int nTargetCount=0;
         double dPwrCalc=Math.abs(dPwr);
         double dAlphaThresh=dAlphaThreshPct*dAlphaBackground;

         int nDistanceRemaining=0, nDistanceTravelled=0;

         switch(nDir) {
             case AUTON_FWD:

             case AUTON_RIGHT:
                 nTargetCount= nStartCount + nDistToMoveEncoderCount;//since moving wheel fwd or right
                 break;
             case AUTON_BACK:
             case AUTON_LEFT:
                 nTargetCount = nStartCount - nDistToMoveEncoderCount;//since moving wheel backwards
                 break;
         }

         setpointNavXYaw(dRobotOrientationToMaintain);

         switch(nDir) {
             case AUTON_FWD:
                 gdDirectionLBPwr =dPwrCalc;
                 gdDirectionLFPwr =dPwrCalc;
                 gdDirectionRFPwr =dPwrCalc;
                 gdDirectionRBPwr =dPwrCalc;
                 break;
             case AUTON_BACK:
                 gdDirectionLBPwr =-dPwrCalc;
                 gdDirectionLFPwr =-dPwrCalc;
                 gdDirectionRFPwr =-dPwrCalc;
                 gdDirectionRBPwr =-dPwrCalc;
                 break;
             case AUTON_LEFT:
                 gdDirectionLFPwr =-dPwrCalc; gdDirectionRFPwr = dPwrCalc;
                 gdDirectionLBPwr = dPwrCalc; gdDirectionRBPwr =-dPwrCalc;
                 break;
             case AUTON_RIGHT:
                 gdDirectionLFPwr = dPwrCalc; gdDirectionRFPwr =-dPwrCalc;
                 gdDirectionLBPwr =-dPwrCalc; gdDirectionRBPwr = dPwrCalc;
                 break;
         }
         while (opMode.opModeIsActive()&& !opMode.isStopRequested() && !Thread.currentThread().isInterrupted()) {
             if ((System.currentTimeMillis() - lTimeStamp) > lMaxTime) {
                 setMecWheelOff();
                 return TIMEDOUT;
             }
             switch(nDir) {
                 case AUTON_FWD:
                 case AUTON_RIGHT:
                     //since moving wheel fwd or right
                     nDistanceRemaining = nTargetCount - mtrLeftFront.getCurrentPosition();
                     nDistanceTravelled = mtrLeftFront.getCurrentPosition() - nStartCount;
                     break;
                 case AUTON_BACK:
                 case AUTON_LEFT:
                     //since moving wheel back or left
                     // target=-100 start =-10 current =-20
                     // target=50  start=100 current=90
                     // target=-20  start =50 current=30 dist=30-(-20)=50
                     // target=-20  start =50 current=-10 dist=-10-(-20)=10
                     nDistanceRemaining = mtrLeftFront.getCurrentPosition()-nTargetCount;
                     // start =50 current=30 dist=50-30=20
                     // target=-20  start =50 current=-10 dist=50-(-10)=60
                     nDistanceTravelled = nStartCount-mtrLeftFront.getCurrentPosition();
                     break;
             }

             switch(nDir) {
                 case AUTON_FWD:
                 case AUTON_RIGHT:
                     if (mtrLeftFront.getCurrentPosition() >= nTargetCount) {
                         setMecWheelOff();
                         return (mtrLeftFront.getCurrentPosition()-nStartCount);
                     }
                     break;
                 case AUTON_BACK:
                 case AUTON_LEFT:
                     if (mtrLeftFront.getCurrentPosition() <= nTargetCount) {
                         setMecWheelOff();
                         return (nStartCount-mtrLeftFront.getCurrentPosition());
                     }
                     break;
             }

             gdRotationPwr = getNavXCorrectionPwr();

             addDirectionAndRotationPwr();

             mtrLeftFront.setPower(gdResultantLFPwr);
             mtrRightFront.setPower(gdResultantRFPwr);
             mtrLeftBack.setPower(gdResultantLBPwr);
             mtrRightBack.setPower(gdResultantRBPwr);

             opMode.telemetry.addData("Direction", "Orientation:%.2f Current %.2f",
                     dRobotOrientationToMaintain, getNavXYaw());
             opMode.telemetry.addData("Pwr", "LF %.2f RF %.2f", gdResultantLFPwr, gdResultantRFPwr);
             opMode.telemetry.addData("   ", "LB %.2f RB %.2f", gdResultantLBPwr, gdResultantRBPwr);
             // Get the normalized colors from the sensor
             NormalizedRGBA colors = colorSensor.getNormalizedColors();

             /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
              * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
              * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
              * for an explanation of HSV color. */

             // Update the hsvValues array by passing it to Color.colorToHSV()
             Color.colorToHSV(colors.toColor(), ghsvValues);

             opMode.telemetry.addLine()
                     .addData("Red", "%.3f", colors.red)
                     .addData("Green", "%.3f", colors.green)
                     .addData("Blue", "%.3f", colors.blue);
             opMode.telemetry.addLine()
                     .addData("Hue", "%.3f", ghsvValues[0])
                     .addData("Saturation", "%.3f", ghsvValues[1])
                     .addData("Value", "%.3f", ghsvValues[2]);
             opMode.telemetry.addData("Alpha", "%.3f Thresh:", colors.alpha,dAlphaThresh);

             opMode.telemetry.update();

             if(colors.alpha>dAlphaThresh) {
                 setMecWheelOff();
                 return LINEFOUND;
             }
         }
         //if got here, something bad happened
         setMecWheelOff();
         return INTERRUPTED;
     }
     //elevator
     public void autonTurnNoNavx(LinearOpMode opMode, double speed, int distance, int timeout) {
         long lMarkMilliS;
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



         int pos = Math.abs(mtrLeftBack.getCurrentPosition());
         lMarkMilliS=System.currentTimeMillis() + timeout;
         while((System.currentTimeMillis() < lMarkMilliS) &&
                 pos < distance &&  opMode.opModeIsActive()
         ) {
             pos = Math.abs(mtrLeftBack.getCurrentPosition());
             mtrLeftBack.setPower(speed);
             mtrLeftFront.setPower(speed);
             mtrRightBack.setPower(-speed);
             mtrRightFront.setPower(-speed);
                 opMode.telemetry.addData( " Elev pos:", "%d",
                         mtrLeftBack.getCurrentPosition());
                 opMode.telemetry.update();
         }
         mtrLeftBack.setPower(0);
         mtrRightBack.setPower(0);
         mtrLeftFront.setPower(0);
         mtrRightFront.setPower(0);
     }
     public void moveRobot(LinearOpMode opMode, String msg, double speed, int distance, int desired_angle, double desired_yaw,
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
             if (desired_angle == 0) {
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
             } else if (msg.equals("turn right")) {
                 msg2 = "turn right";
                 mtrLeftBack.setPower(myspeed);
                 mtrLeftFront.setPower(myspeed);
                 mtrRightBack.setPower(-myspeed);
                 mtrRightFront.setPower(-myspeed);
             } else if (msg.equals("turn right")) {
                 msg2 = "turn right";
                 mtrLeftBack.setPower(speed);
                 mtrLeftFront.setPower(speed);
                 mtrRightBack.setPower(-speed);
                 mtrRightFront.setPower(-speed);
             } else if (msg.equals("turn left")) {
                 msg2 = "turn left";
                 mtrLeftBack.setPower(-speed);
                 mtrLeftFront.setPower(-speed);
                 mtrRightBack.setPower(speed);
                 mtrRightFront.setPower(speed);
             }
             else {
                 msg2 = "no movement";
             }
             opMode.telemetry.addData(msg2 + " "+ msg + " M pos:", "%.2f, %d, %d, %d, %d", heading,
                     mtrLeftBack.getCurrentPosition(), mtrRightBack.getCurrentPosition(),
                     mtrRightFront.getCurrentPosition(), mtrLeftFront.getCurrentPosition());
             opMode.telemetry.update();

         }
         mtrLeftBack.setPower(0);
         mtrRightBack.setPower(0);
         mtrLeftFront.setPower(0);
         mtrRightFront.setPower(0);

     }
     public void getNavx (OpMode opMode) {
         opMode.telemetry.addData( " getNavx:", "%f",
                 navX.getYaw());
         opMode.telemetry.update();
     }

}


