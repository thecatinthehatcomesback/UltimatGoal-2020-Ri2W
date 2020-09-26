/*
        CatHW_Subsystem.java

    A "hardware" class containing common code accessing hardware objects
    and processes.  It detects if the subclasses are busy and can/should
    continue to the next step/segment of code.  This file is used by
    CatHW_Async to run multiple processes at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode.
 *
 * This class is used to detect the different subclasses business and
 * whether they can/should continue to the next step/segment of code.
 * This file is used by CatHW_Async to run multiple processes at once.
 */
public class CatHW_Subsystem
{

    /* local OpMode members. */
    public HardwareMap hwMap        = null;
    public CatHW_Async mainHW        = null;

    /* Constructor */
    public CatHW_Subsystem(CatHW_Async mainHardware){

        mainHW = mainHardware;
        hwMap  = mainHW.hwMap;

    }
    public boolean isDone (){
        return true;
    }

    public boolean isBusy (){
        return !isDone();
    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException {

    }

    public void waitUntilDone(){
        while (!isDone()) {
            if (!(mainHW.opMode.opModeIsActive())){
                return;
            }
        }
    }


    public static void waitUntilDone(CatHW_Subsystem subOne, CatHW_Subsystem subTwo){
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();
        while (subOneBusy || subTwoBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())){
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
        }
    }

    public static void waitUntilDone(CatHW_Subsystem subOne, CatHW_Subsystem subTwo, CatHW_Subsystem subThree){
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();
        boolean subThreeBusy = subThree.isBusy();
        while (subOneBusy || subTwoBusy || subThreeBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())){
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
            subThreeBusy = subThree.isBusy();
        }
    }

    /**
     * ---   __________________   ---
     * ---   End of Our Methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket