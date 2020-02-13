package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;
    String id;
    public Limelight(String name) {
        id = name;
        table = NetworkTableInstance.getDefault().getTable("limelight-" + id);
    }

    public double tx() {
        return  table.getEntry("tx").getDouble(0.0);
    }

    public double ty() {
        return  table.getEntry("ty").getDouble(0.0);
    }

    public double[] getDistance() {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double roll = 0.0;
        double[] distances = {x, y, z, pitch, yaw, roll};

        x = table.getEntry("camtran").getDoubleArray(distances)[0];
        y = table.getEntry("camtran").getDoubleArray(distances)[1];
        z = table.getEntry("camtran").getDoubleArray(distances)[2];

        distances[0] = x; //Maybe add a *0.8 back in if it helps
        distances[1] = y;
        distances[2] = z;

        return  distances;
    }

    public void enable() {
        table.getEntry("ledMode").setNumber(0);
    }

    public void disable() {
        table.getEntry("ledMode").setNumber(1);
    }
}
