package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CatHW_Arm extends CatHW_Subsystem {

    ElapsedTime runTime = new ElapsedTime();
    double      timeout = 0;

    public DcMotor armMotor = null;
    public DcMotor intake  = null;

    LinearOpMode opMode = null;

    public CatHW_Arm(CatHW_Async mainHardware) { super(mainHardware); }

    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        armMotor    = hwMap.get(DcMotor.class,"arm");
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        intake = hwMap.get(DcMotor.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes //
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors to run at no power so that the robot doesn't move during init //
        armMotor.setPower(0);
        intake.setPower(0);




    }

    public void intakeRings(double power){
        intake.setPower(power);

    }

    public void stopIntake(){
        intake.setPower(0);

    }

    public void raiseArm(double time){
        armMotor.setPower(0.6);
        robotWait(time);
        armMotor.setPower(0);
    }


    public void lowerArm(double time){
        armMotor.setPower(-0.6);
        robotWait(time);
        armMotor.setPower(0);
    }

    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (delayTimer.seconds() < seconds) {
            //while (opMode.opModeIsActive()  &&  (delayTimer.seconds() < seconds)) {
        //    opMode.idle();
        }
    }

}
