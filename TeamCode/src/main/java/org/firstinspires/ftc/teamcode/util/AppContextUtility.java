package org.firstinspires.ftc.teamcode.util;

import android.app.Application;
import android.content.Context;
import android.content.res.AssetManager;

/**
 * This class provides a way to get access to the assets folder through
 * an additional application.
 */
public class AppContextUtility extends Application {
    private static Context mContext;

    @Override
    public void onCreate() {
        super.onCreate();
        mContext = this;
    }

    public static Context getContext(){
        return mContext;
    }
    public static AssetManager getAssetManager(){
        if (mContext != null)
            return mContext.getAssets();
        else
            return null;
    }
}
