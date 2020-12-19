package org.firstinspires.ftc.teamcode.util;

import android.app.Application;
import android.content.Context;
import android.content.res.AssetManager;

/**
 * This class provides a way to get access to the assets folder through
 * an additional application.
 */
public class AppContextUtility {
    private static Context mContext = null;

    /**
     * This function is called from the FtcRobotControllerActivity.  That code must be modified
     * manually on each updated during the season.
     * @param context
     */
    public static void setApplicationContext(Context context){
        mContext = context;
    }

    public static AssetManager getAssetManager(){
        if (mContext != null)
            return mContext.getAssets();
        else
            return null;
    }
}
