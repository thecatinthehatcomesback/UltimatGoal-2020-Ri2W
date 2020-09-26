/*
    Mec_TeleOpLevel5_Statey.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.

    By FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Ri2W TeleOp", group = "CatTeleOp")
public class Ri2W_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    private ElapsedTime stoneReleaseTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot = null;  // Use our new mecanum async hardware


    public DcMotorSimple intake  = null;
    public DcMotorSimple arm  = null;

    // Our constructor for this class
    public Ri2W_TeleOp() {
        robot = new CatHW_Async();

    }

    @Override
    public void runOpMode() throws InterruptedException {


        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this, false);
        // Finished!  Now tell the driver...

        intake = hardwareMap.get(DcMotorSimple.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        arm    = hardwareMap.get(DcMotorSimple.class,"arm");
        arm.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        stoneReleaseTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;


        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.30;
            } else {
                driveSpeed = 0.70;
            }

            // Input for setDrivePowers train and sets the dead-zones:
            leftFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;
            leftBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;


            // Calculate the scale factor:
            SF = robot.driveClassic.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;
            // DRIVE!!!
            robot.driveClassic.drive(leftFront, rightFront, leftBack, rightBack);



            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */
            intake.setPower((gamepad2.right_trigger -gamepad2.left_trigger)*0.7);
            arm.setPower(-gamepad2.right_stick_y * 0.7);

            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            //telemetry.addData("Intake Power:","%.2f", robot.jaws.leftJawMotor.getPower());

            telemetry.addData("Encoder lf/lr rf/rr", "%5d %5d   %5d %5d",
                    robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                    robot.driveClassic.leftRearMotor.getCurrentPosition(),
                    robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                    robot.driveClassic.rightRearMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
