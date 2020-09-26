/*
        CatDriveHW.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the drive train.  This is a modified
    or stripped down version of CatSingleOverallHW to run all of intake
    movements.  This file is used by the new autonomous OpModes to run
    multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the specific hardware for the robot to
 * allow for multiple operations during autonomous.  In this case, that robot is //todo Change this name
 * Jack from the Cat in the Hat Comes Back team during the 2018-2019 season.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_DriveClassic extends CatHW_Subsystem
{
    /* Wheel measurements */
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // Accurate for a NeveRest Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Autonomous Drive Speeds */
    static final double     HYPER_SPEED             = 0.95;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     CHILL_SPEED             = 0.4;
    static final double     CREEP_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.6;


    ElapsedTime runTime = new ElapsedTime();
    double      timeout = 0;

    double          targetX;
    double          targetY;
    double          strafePower;
    double          strafeAngle;
    double          strafeTurnPower;

    int             targetAngleZ;
    int             baseDelta;
    boolean         clockwiseTurn;
    ColorSensor     leftColSen;
    ColorSensor     rightColSen;
    static boolean  isDone;

    /* Enums */
    enum DRIVE_METHOD {
        vertical,
        horizontal,
        turn,
        strafe
    }

    enum DRIVE_MODE {
        followWall,
        driveTilDistance,
        driveUsingGyroStraight
    }

    enum TURN_MODE {
        SPIN,
        TANK
    }


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private DRIVE_METHOD currentMethod;
    private DRIVE_MODE currentMode;

    /* Public OpMode members. */
    // Motors
    public DcMotor  leftFrontMotor  = null;
    public DcMotor  rightFrontMotor = null;
    public DcMotor  leftRearMotor   = null;
    public DcMotor  rightRearMotor  = null;




    /* local OpMode members. */
    LinearOpMode opMode = null;
    /* Constructor */
    public CatHW_DriveClassic(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");


        // Define motor directions //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // Define motor zero power behavior //
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set motor modes //
        runNoEncoders();


        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);


        // Sets enums to a default value
        currentMode = DRIVE_MODE.driveTilDistance;
        currentMethod = DRIVE_METHOD.vertical;

    }


    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    /* Basic methods for setting all four drive motor powers and setModes: */
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        /**
         * Simply setting the powers of each motor in less characters
         */
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);

        Log.d("catbot", String.format("Drive Power  LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFront, rightFront, leftBack, rightBack));
    }
    public void resetEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runNoEncoders(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runToPosition(){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Driving Methods:
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /**
         * Will scale down our power numbers if they are
         * greater than 1.0 so that we continue the set
         * course and don't just limit to the highest
         * possible value...
         */

        /**
         * Look at all motor values
         * Find the highest absolute value (the "scalor")
         * If the highest value is not more than 1.0, we don't need to change the values
         * But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
         * Finally, we pass out the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value
        for(int i = 0; i+1 < values.length; i++){
            if(values[i] > scalor){
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done here...
        if (scalor > 1.0) {
            // Get the reciprocal
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don'thave to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }
    public void mecDriveVertical(double power,
                                 double distance,
                                 double timeoutS)  throws InterruptedException {
        mecDriveVertical(power, distance, timeoutS, DRIVE_MODE.driveTilDistance, null, null);
    }
    public void mecDriveVertical(double power,
                                 double distance,
                                 double timeoutS, DRIVE_MODE driveMode, ColorSensor leftColSenIn, ColorSensor rightColSenIn)  throws InterruptedException {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight vertically or using the color sensors to find a
         * line.
         */

        Log.d("catbot", String.format(" Started drive vert pow: %.2f, dist: %.2f, time:%.2f ",power,distance, timeoutS));
        currentMethod = DRIVE_METHOD.vertical;
        currentMode = driveMode;
        timeout = timeoutS;
        leftColSen = leftColSenIn;
        rightColSen = rightColSenIn;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        boolean keepDriving = true;
        baseDelta = 0;
        isDone = false;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget  = (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget  = (int) (distance * COUNTS_PER_INCH);

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards
            if (distance < 0) {
                power = -power;
            }

            drive(power, power, power, power);

        }
    }
    public void mecDriveHorizontal(double power, double distance, double timeoutS) {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight horizontally (positive numbers should strafe left)
         */
        Log.d("catbot", String.format(" Started drive horizontal pow: %.2f, dist: %.2f, time:%.2f ",power,distance, timeoutS));

        currentMethod = DRIVE_METHOD.horizontal;
        timeout = timeoutS;
        isDone = false;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // (Multiply by sqrt of 2 to compensate)
            newLeftFrontTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightBackTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going right
            if (distance < 0) {
                power = -power;
            }

            // Due to the differences in weight on each wheel, adjust powers accordingly
            drive(power, power, power, power);
        }
    }
    public void advMecDrive(double power, double vectorDistance,
                            double vectorAng, double timeoutS) throws InterruptedException {
        /**
         * In this mecanum drive method, we are trying to have the robot
         * drive at an angle while the face of the robot remains pointed
         * ahead.
         *
         *
         * / = back left and forward right motors
         * \ = back right and forward front motors
         *
         * We add the Sin of our angle to the Cos in order to get the
         * powers for \ side of motors while we subtract Sin from Cos
         * for the / side of motors.
         */

        double leftFrontMod  = Math.sin(Math.toRadians(vectorAng))  + Math.cos(Math.toRadians(vectorAng));
        double rightFrontMod = -Math.sin(Math.toRadians(vectorAng)) + Math.cos(Math.toRadians(vectorAng));
        double leftBackMod   = -Math.sin(Math.toRadians(vectorAng)) + Math.cos(Math.toRadians(vectorAng));
        double rightBackMod  = Math.sin(Math.toRadians(vectorAng))  + Math.cos(Math.toRadians(vectorAng));


        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        boolean keepDriving = true;
        isDone = false;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position and multiply each one to adjust for variation of mec wheels
            newLeftFrontTarget  = (int) (vectorDistance * COUNTS_PER_INCH * leftFrontMod);
            newRightFrontTarget = (int) (vectorDistance * COUNTS_PER_INCH * rightFrontMod);
            newLeftBackTarget   = (int) (vectorDistance * COUNTS_PER_INCH * leftBackMod);
            newRightBackTarget  = (int) (vectorDistance * COUNTS_PER_INCH * rightBackMod);

            // Set the motors to travel towards their desired targets
            resetEncoders();
            runToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards
            if (vectorDistance < 0) {
                power = -power;
            }

            // Calculate motor drive powers after we decide direction
            double SF = findScalor(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);
            leftFrontMod  = leftFrontMod  * SF * power;
            rightFrontMod = rightFrontMod * SF * power;
            leftBackMod   = leftBackMod   * SF * power;
            rightBackMod  = rightBackMod  * SF * power;
            // Drive
            drive(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);

            while (opMode.opModeIsActive() &&
                    (runTime.seconds() < timeoutS) &&
                    keepDriving) {

                // Find the current positions so that we can display it later
                int leftFrontPosition  = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition   = leftRearMotor.getCurrentPosition();
                int rightBackPosition  = rightRearMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftRearMotor.isBusy() && !rightRearMotor.isBusy()) {
                    keepDriving = false;
                }

                // Display it for the driver
                opMode.telemetry.addData("New Path",  "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",  "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runTime.seconds());
                opMode.telemetry.update();
            }


            // Stop all motion
            drive(0, 0, 0, 0);
        }
    }
    public void mecTurn(double power, int degrees, double timeoutS) throws InterruptedException {
        mecTurn(power, degrees, timeoutS, TURN_MODE.SPIN);
    }
    public void mecTurn(double power, int degrees, double timeoutS, TURN_MODE turnMode) throws InterruptedException {
        /**
         * Turns counterclockwise with a positive Z angle
         * Turns clockwise with a negative Z angle
         */

        currentMethod = DRIVE_METHOD.turn;
        timeout = timeoutS;
        isDone = false;

        // Ensure that the opMode is still active
        if (mainHW.opMode.opModeIsActive()) {
            targetAngleZ  = -degrees;
            clockwiseTurn = (getCurrentAngle() > targetAngleZ);

            // Don't use encoders.  We only use the gyro angle to turn
            runNoEncoders();
            // reset the timeout time and start motion.
            runTime.reset();
            Log.d("catbot", String.format("Start turn...  target %d, current %d  %s", -targetAngleZ, -getCurrentAngle(), clockwiseTurn ?"CW":"CCW"));


            // Change the power based on which angle we are turning to
            if (clockwiseTurn) {
                leftFrontMotor.setPower(power);
                leftRearMotor.setPower(power);
                if (turnMode == TURN_MODE.SPIN) {
                    rightFrontMotor.setPower(-power);
                    rightRearMotor.setPower(-power);
                } else {
                    rightFrontMotor.setPower(-power/3);
                    rightRearMotor.setPower(-power/3);
                }
            } else {
                if (turnMode == TURN_MODE.SPIN) {
                    leftFrontMotor.setPower(-power);
                    leftRearMotor.setPower(-power);
                }  else {
                    leftFrontMotor.setPower(-power/3);
                    leftRearMotor.setPower(-power/3);
                }
                rightFrontMotor.setPower(power);
                rightRearMotor.setPower(power);
            }

        }
    }

    public void strafeDrive(double x, double y, double power, double angle, double turnSpeed, double timeoutS){

        currentMethod = DRIVE_METHOD.strafe;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        strafeAngle = angle;
        strafeTurnPower = turnSpeed;



        runTime.reset();
    }

    /**
     * ---   ___________   ---
     * ---   IMU Methods   ---
     * ---   \/ \/ \/ \/   ---
     */
    public void IMUinit () {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        //the initialize method is taking a whole second
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
    }
    public int getCurrentAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }

    /**
     * ---   _____________   ---
     * ---   isDone Method   ---
     * ---  \/ \/ \/ \/ \/   ---
     */
    @Override
    public boolean isDone() {
        boolean keepDriving = true;
        if (runTime.seconds() > timeout) {
            // Log message:
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }
        switch (currentMethod){
            case vertical:
                // One setDrivePowers mode that drives blindly straight
                if (currentMode == DRIVE_MODE.driveTilDistance) {

                    //  Exit the method once robot stops
                    if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                            !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                        keepDriving = false;
                    }
                    // Log message:
                    Log.d("catbot", String.format("DriveVert LF: %d, %d;  RF: %d, %d;  LB: %d, %d;  RB %d,%d",
                            leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(),
                            leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition()));
                }

                break;

            case horizontal:
                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                        !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {

                    keepDriving = false;
                }
                // Log message:
                Log.d("catbot", String.format("DriveHor LF: %d, %d, %.2f;  RF: %d, %d, %.2f;  LB: %d, %d, %.2f;  RB %d,%d, %.2f",
                        leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),  leftFrontMotor.getPower(),
                        rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(), rightFrontMotor.getPower(),
                        leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(), leftRearMotor.getPower(),
                        rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition(), rightRearMotor.getPower()));

                break;

            case turn:

                int zVal = getCurrentAngle();

                // Log message:
                Log.d("catbot", String.format("turn  target %d, current %d  %s", targetAngleZ, zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                break;
        }

        if (!keepDriving){
            // Stop all motion
            drive(0, 0, 0, 0);
            isDone = true;
            return true;
        }
        if (isDone){
            return true;
        }
        return false;
    }
    /**
     * ---   __________________   ---
     * ---   End of Our Methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket