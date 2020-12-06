package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Encapsulates a logging file for development
 */
public class LogFile {

    private String[] mColumnNames;
    private String mFilePath;
    private String mFilename;
    private File mLogFile;
    private BufferedWriter mWriter;

    public LogFile(String filePath, String filename, String[] columnNames){
        File root = Environment.getRootDirectory();
        mFilePath = root.getPath()+"/"+filePath;
        mFilePath = filePath;
        mColumnNames = columnNames;
        mFilename = filename;
    }

    public void openFile(){
        try {
            mLogFile = new File(mFilePath,mFilename);
            if (mLogFile.exists()){
                mLogFile.delete();
            }
            FileWriter fw = new FileWriter(mLogFile);
            mWriter = new BufferedWriter(fw);
            for(int i=0;i < mColumnNames.length;i++){
                mWriter.write(mColumnNames[i]);
                if (i != (mColumnNames.length-1)) {
                    mWriter.write(',');
                }
            }
            mWriter.newLine();
        }
        catch(IOException e){
            e.printStackTrace();
            return;
        }
    }

    public void closeFile(){
        try{
            mWriter.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public void writeLogRow(String[] data){
        if (data.length != mColumnNames.length){
            throw new IllegalArgumentException("data length="+data.length+".  Must be equal to number of columns:"+mColumnNames.length);
        }
        try {
            for (int i = 0; i < mColumnNames.length; i++) {
                mWriter.write(data[i]);
                if (i != (mColumnNames.length - 1)) {
                    mWriter.write(',');
                }
            }
            mWriter.newLine();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }
}
