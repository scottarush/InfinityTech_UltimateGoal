package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Encapsulates an option selected via the joystick.
 */
public class GamepadMenu {
    protected OpMode mOpMode = null;

    private String[] mOptionStrings = new String[1];
    private String mCaption = "";
    private int mSelectedOption = 0;
    private Gamepad mGamepad = null;

    public GamepadMenu(OpMode opMode, Gamepad gamepad,String caption, String[] options){
        mOpMode = opMode;
        mCaption = caption;
        mOptionStrings = options;
        mGamepad = gamepad;
    }

    /**
     * Overriden by subclasses to add the specific menu items
     * @return true when the option has been selected.
     */
    public boolean doOptionMenu(){
        if (mGamepad.b){
            return true;
        }
        if (mGamepad.dpad_up){
            mSelectedOption++;
            if (mSelectedOption > mOptionStrings.length-1){
                mSelectedOption = 0;
            }
        }
        else if (mGamepad.dpad_down){
            mSelectedOption--;
            if (mSelectedOption < 0){
                mSelectedOption = 0;
            }
        }
        // Show the currently option
        mOpMode.telemetry.addData(mCaption, mOptionStrings[mSelectedOption]);
        mOpMode.telemetry.update();
        return false;
    }

    /**
     * returns the option that was selected.
     */
    public int getSelectedOption(){
        return mSelectedOption;
    }
}
