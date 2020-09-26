/*
        CatHW_Async.java

    An "hardware" class that acts as the master in which all the other
    "hardwares" run through.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the other hardwares.
 *
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  setDrivePowers motor:        "left_rear"  & "left_front"
 * Motor channel:  Right setDrivePowers motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_Async
{
    /* Public OpMode members. */
    public boolean isInitOdo = false;


    public static boolean isRedAlliance = true;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;


    /* Other Hardware subSystems */

    CatHW_DriveClassic  driveClassic    = null;
    CatHW_Arm           arm             = null;

    /* Constructor */
    public CatHW_Async(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode, boolean isInitOdo)  throws InterruptedException  {

        // Save Reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        // Give Telemetry for each system we begin to init:
        opMode.telemetry.addData("Initialize","DriveClassic...");
        opMode.telemetry.update();
        driveClassic = new CatHW_DriveClassic(this);
        arm = new CatHW_Arm(this);
        driveClassic.init();
        arm.init();


        opMode.telemetry.addData("Initialize","All Done...  BOOM!");
        opMode.telemetry.update();

    }

    /**
     * ---   ____________________________   ---
     * ---   Common Miscellaneous Methods   ---
     * ---  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/   ---
     */

    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive()  &&  (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
    }
    public double limitRange(double number, double min, double max) {
        return Math.min(Math.max(number, min), max);
    }

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket