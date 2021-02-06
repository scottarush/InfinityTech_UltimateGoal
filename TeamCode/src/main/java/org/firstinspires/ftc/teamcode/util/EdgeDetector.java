package org.firstinspires.ftc.teamcode.util;

public class EdgeDetector {

    private boolean mLastState = false;
    public EdgeDetector(){

    }

    /**
     *
     * @return true on rising edge
     */
    public boolean sampleRisingEdge(boolean state){
        boolean risingEdge = false;
        if (state != mLastState){
            if (state){
                risingEdge = true;
            }
        }
        mLastState = state;
        return risingEdge;
    }
}
