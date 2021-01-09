package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;

public interface ICaptureCameraListener {
    /**
     * Called on receipt of a new receipt bitmap from the camera. If consuming the bitmap
     * for processing, then receivers must copy the bitmap before returning as the supplied
     * bitmap buffer will be recycled upon return.
     *
     * @param bitmap the new bitmap
     **/
    void onNewFrame(Bitmap bitmap);
}
