package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineSensor {
    private static DigitalInput left = new DigitalInput(8);
    private static DigitalInput right = new DigitalInput(9);

    public static int get() {
        int command = 0;

        if (left.get() && right.get()) { //Don't move because sensors found line
            command = 3;
        } else if (left.get()) { //Move robot left
            command = 1;
        } else if (right.get()) { //Move robot right
            command = 2;
        } else {
            command = 0; //Couldn't find line
        }

        SmartDashboard.putBoolean("Line Left", left.get());
        SmartDashboard.putBoolean("Line Right", right.get());
        SmartDashboard.putBoolean("Cargo Aligned", (left.get() && right.get()));
        return command;
    }
}
