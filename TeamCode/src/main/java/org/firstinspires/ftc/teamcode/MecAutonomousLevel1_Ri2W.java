/**
 MecAutonomousLevel1_Ri2W.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of with gamepad1 and
 detect the gold with DogeCV, hit the right element, place our team marker
 in our depot and park in either crater according to what we say using gamepad1
 at the beginning of the match.

 MecBasic is written to use the most basic approach to our autonomous route
 with the help of mechanical sorting intake and a servo to drop our team marker
 off in the depot.  This autonomous is used for our first qualifier this year
(November 10, 2018).

 This was the old code for our Robot in 2 Weeks bot, but we completely stripped
 it out as with our old bot.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Ri2W Autonomous", group="CatAuto")
public class MecAutonomousLevel1_Ri2W extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    //CatVisionHW eyes  = new CatVisionHW();   // Vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = true;
    private boolean isParkAtWall = false;

    //private CatVisionHW.samplingPos samplingPos = CatVisionHW.samplingPos.RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the drive system variables.  The init() methods of
         * our hardware class does all the work:
         */
        robot.init(hardwareMap, this,false);
        // Init IMU sensor later when the match starts to avoid drifting in the values
        // Init our Machine Vision
        //eyes.initVision(hardwareMap);

        /**
         * Send telemetry message to signify robot getting ready:
         */
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();
        //robot.drive.resetEncoders();
        //idle();
        //robot.drive.runToPosition();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :%7d  :%7d",
                robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                robot.driveClassic.leftRearMotor.getCurrentPosition(),
                robot.driveClassic.rightRearMotor.getCurrentPosition());
        telemetry.update();

        /**
         * Init Delay Option Select:
         */
        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.y) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isBuildZone) {
                    isBuildZone = false;
                } else {
                    isBuildZone = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isParkAtWall) {
                    isParkAtWall = false;
                } else {
                    isParkAtWall = true;
                }
                delayTimer.reset();
            }

            /**
             * LED code:
             */
            if (isRedAlliance) {

            } else {

            }

            /**
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            telemetry.addData("isBuildZone Side?", isBuildZone);
            telemetry.addData("isPartAtWall", isParkAtWall);
            telemetry.update();

            /**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }
        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */
        robot.driveClassic.IMUinit();

        /* Go! */
        Stuff();
    }




    public void Stuff() throws InterruptedException {

            robot.driveClassic.mecDriveVertical(0.6, -118, 4);
            robot.driveClassic.waitUntilDone();
            robot.robotWait(.5);
            robot.driveClassic.mecDriveVertical(0.6, 6, 3);
            robot.driveClassic.waitUntilDone();
            robot.driveClassic.mecDriveHorizontal(0.6, -34, 3);
            robot.driveClassic.waitUntilDone();
            robot.driveClassic.mecTurn(0.6, 165, 5);
            robot.driveClassic.waitUntilDone();
            robot.arm.raiseArm(0.7);

            // Drive ahead and dump the rings.
            robot.driveClassic.mecDriveVertical(0.6, 33, 3);
            robot.driveClassic.waitUntilDone();
            robot.arm.intakeRings(-0.8);
            robot.robotWait(5);
            robot.arm.stopIntake();

            robot.robotWait(.5);
            // Back up to park on line.
            robot.driveClassic.mecDriveVertical(0.6, -44, 3);
            robot.driveClassic.waitUntilDone();
            //robot.driveClassic.mecDriveHorizontal(0.6, -24, 3);
            //robot.driveClassic.waitUntilDone();

    }
}
