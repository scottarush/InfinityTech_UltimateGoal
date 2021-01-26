package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class ImageUtils {

    public static void savePNG(Bitmap bitmap, File filePath){
        try {
            try (FileOutputStream outputStream = new FileOutputStream(filePath)) {
                bitmap.compress(Bitmap.CompressFormat.PNG,100, outputStream);
            }
        } catch (IOException e) {
            RobotLog.ee("ImageUtils.saveBitmap", e, "exception in saveBitmap()");

        }
    }
}
