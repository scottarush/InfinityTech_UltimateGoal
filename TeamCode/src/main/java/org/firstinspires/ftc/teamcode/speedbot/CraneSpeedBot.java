package org.firstinspires.ftc.teamcode.speedbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is the Speed Bot that includes the crane
 * -------------------------------------------------
 * HUB Layout:
 * -------------------------------------------------
 * Port0:  left front wheel hex motor "lf"
 * Port1:  right front wheel hex motor "rf"
 * Port2:  left rear wheel hex motor "lr"
 * Port3:  right rear wheel hex motor "rr"
 *
 * Camera is "webcam"
 */
public class CraneSpeedBot extends BaseSpeedBot {

    protected OpMode mOpMode;

    private boolean mEnableIMU = false;

    private Crane mCrane = null;


    public CraneSpeedBot(OpMode opMode, boolean enableIMU){
        super(opMode, enableIMU);
         mCrane = new Crane(opMode);
     }

    public Crane getCrane(){
        return  mCrane;
    }

     /**
     *
     * @throws Exception
     */
    public void init() throws Exception {
        String initErrString = "";
        try {
           super.init(null);
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        try{
            mCrane.init(mOpMode.hardwareMap);
        }
        catch (Exception e){
            initErrString += e.getMessage();
        }
        if (initErrString.length() > 0){
            throw new Exception(initErrString);
        }
    }

}
