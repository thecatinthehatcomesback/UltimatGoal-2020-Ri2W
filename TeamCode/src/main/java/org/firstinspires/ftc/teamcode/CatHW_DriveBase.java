/*
        CatHW_DriveBase.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the setDrivePowers train.  This is a
    modified or stripped down version of CatSingleOverallHW to run all
    the drive train overall.  This file is used by the new autonomous
    OpModes to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * This class is used to define all the specific and basic hardware for the
 * robot's drive train and such to allow for multiple operations during
 * autonomous.  This class has two subclasses (CatHW_DriveClassic and
 * CatHW_DriveOdo) that use encoders to determine position differently.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  setDrivePowers motor:        "left_rear"  & "left_front"
 * Motor channel:  Right setDrivePowers motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_DriveBase extends CatHW_Subsystem
{
    /* Wheel measurement constants */
    private static final double COUNTS_PER_REVOLUTION   = 537.6; // Accurate for NeveRest Orbital 20
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;   // For calculating circumference
    static final double         COUNTS_PER_INCH         = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);


    // Timer stuff
    ElapsedTime runTime = new ElapsedTime();
    double      timeout = 0;

    // Turn stuff
    int         targetAngleZ;
    int         baseDelta;
    boolean     clockwiseTurn;

    // isDone stuff
    static boolean  isDone;


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // LED stuff
    public RevBlinkinLedDriver lights   = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    /* Public OpMode members. */
    // Motors:
    public DcMotor leftFrontMotor  = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor   = null;
    public DcMotor rightRearMotor  = null;

    /* local OpMode members. */
    LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveBase(CatHW_Async mainHardware) { super(mainHardware); }


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
        setDriveToBrake();

        // Set motor modes //
        resetDriveEncoders();
        setDriveRunWithoutEncoders();

        // Set all motors to run at no power so that the robot doesn't move during init //
        setDrivePowers(0,0, 0, 0);


    }

    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    // Basic methods for setting all four setDrivePowers motor powers and setModes:
    /**
     * Sets powers to the four drive train motors
     * @param leftFront motor's power
     * @param rightFront motor's power
     * @param leftBack motor's power
     * @param rightBack motor's power
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);

        // Log message:
        //Log.d("catbot", String.format("Drive Power  LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFront, rightFront, leftBack, rightBack));
    }
    /**
     * Set drive train motors to BRAKE
     */
    public void setDriveToBrake() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * Set drive train motors to FLOAT (coast)
     */
    public void setDriveToCoast() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    /**
     * Set drive train motors to STOP_AND_RESET_ENCODER
     */
    public void resetDriveEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * Set drive train motors to RUN_USING_ENCODER
     */
    public void setDriveRunUsingEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Set drive train motors to RUN_WITHOUT_ENCODER
     */
    public void setDriveRunWithoutEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * Set drive train motors to RUN_TO_POSITION
     */
    public void setDriveRunToPosition(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * ---   ___________   ---
     * ---   IMU Methods   ---
     * ---   \/ \/ \/ \/   ---
     */
    public void IMU_Init() {
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
        return -(int)angles.firstAngle;
    }

    // Mathematical operations:

    /**
     * Will scale down our calculated power numbers if they are greater
     * than 1.0.  If the values were greater than 1.0, the motors would
     * spin at their max powers.  This would limit precise paths the
     * robot could take, thus we created this method to "scale down" all
     * the values by creating a scale factor so that there is a
     * proportional difference in all the motor powers, giving the robot
     * better mobility, especially with mecanum wheels.
     *
     * @param leftFrontValue Prospective value for motor power that may be scaled down
     * @param rightFrontValue Prospective value for motor power that may be scaled down
     * @param leftBackValue Prospective value for motor power that may be scaled down
     * @param rightBackValue Prospective value for motor power that may be scaled down
     * @return scaleFactor The double by which we'd multiply all the other motor powers by
     */
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /**
         * Plan:
         *
         * 1: Look at all motor values
         * 2: Find the highest absolute value (the "scalor")
         * 3: If the highest value is not more than 1.0, we don't need to change the values
         * 4: But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
         * 5: Finally, we pass OUT the scale factor so that we can scale each motor down
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
            // Set to 1 so that we don't change anything we don't have to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }

    /**
     * ---   __________________   ---
     * ---   End of Our Methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket