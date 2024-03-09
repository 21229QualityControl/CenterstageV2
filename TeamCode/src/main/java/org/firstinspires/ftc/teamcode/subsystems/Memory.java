package org.firstinspires.ftc.teamcode.subsystems;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.util.DualMotorWithPID;
import org.firstinspires.ftc.teamcode.util.MotorWithPID;

/**
 * Static class to transition auto values into manual
 */
//@Config
public class Memory {
    private static final String memoryDirectory = "/FIRST/data";
    public static boolean RAN_AUTO = false;
    public static boolean FINISHED_AUTO = true;
    public static Pose2d LAST_POSE = new Pose2d(0, 0, 0);

    // For time persistence
    public static String SAVED_TIME_FILE_NAME = "AutoStartTime";

    /**
     * Saves a string to a txt file.
     * Replaces the previous file if it exists.
     * @param value the string to save to file
     * @param fileName the name of the saved file without extension
     */
    public static void saveStringToFile(String value, String fileName) {
        // Check if we have read/write access
        if (!Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())) {
            Log.e("SaveStringToFile", "External storage not available for writing");
            return;
        }
        File file = new File(Environment.getExternalStorageDirectory() + memoryDirectory, fileName + ".txt");

        // Check if the file already exists
        if (file.exists()) {
            file.delete();
        }

        try {
            FileWriter writer = new FileWriter(file);
            writer.write(value);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Returns the string in a txt file.
     *
     * @param fileName the name of the file to read without the extension
     * @return the string in the file or null if file does not exist
     */
    public static String getStringFromFile(String fileName) {
        // Check if we have read/write access
        if (!Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())) {
            Log.e("GetStringFromFile", "External storage not available for reading");
            return null;
        }

        try {
            File file = new File(Environment.getExternalStorageDirectory() + memoryDirectory, fileName + ".txt");
            Scanner reader = new Scanner(file).useDelimiter("\\A");
            String output = reader.next();
            reader.close();
            return output;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
