package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class Storage {
    private static final File STORAGE_FILE = new File(AppUtil.FIRST_FOLDER, "robot_state.txt");

    public static class Data {
        public final ComponentShell.Alliance storedAlliance;
        public final Pose storedPose;

        public Data(ComponentShell.Alliance al, Pose storedPose) {
            this.storedAlliance = al;
            this.storedPose = storedPose;
        }
    }

    public static void write(ComponentShell.Alliance b, Pose p) {
        STORAGE_FILE.getParentFile().mkdirs();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(STORAGE_FILE))) {
            if (b == ComponentShell.Alliance.RED) {
                writer.write(Boolean.toString(false));
            } else {
                writer.write(Boolean.toString(true));
            }
            writer.newLine();
            writer.write(Double.toString(p.getX()));
            writer.newLine();
            writer.write(Double.toString(p.getY()));
            writer.newLine();
            writer.write(Double.toString(p.getHeading()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Data read() {
        if (!STORAGE_FILE.exists()) {
            return new Data(ComponentShell.Alliance.BLUE, new Pose(0, 0, 0)); // Default values
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(STORAGE_FILE))) {
            boolean b = Boolean.parseBoolean(reader.readLine());
            ComponentShell.Alliance al = ComponentShell.Alliance.RED;
            if (b) {
                al = ComponentShell.Alliance.BLUE;
            }
            double x = Double.parseDouble(reader.readLine());
            double y = Double.parseDouble(reader.readLine());
            double h = Double.parseDouble(reader.readLine());
            return new Data(al, new Pose(x, y, h));
        } catch (IOException | NumberFormatException | NullPointerException e) {
            e.printStackTrace();
            // On error, return default values
            return new Data(ComponentShell.Alliance.BLUE, new Pose(0, 0, 0));
        }
    }
}
