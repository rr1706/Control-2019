package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineSensor {
    private static DigitalInput frontRight = new DigitalInput(0);
    private static DigitalInput frontMid = new DigitalInput(1);
    private static DigitalInput frontLeft = new DigitalInput(2);
    private static DigitalInput backLeft = new DigitalInput(3);
    private static DigitalInput backMid = new DigitalInput(6);
    private static DigitalInput backRight = new DigitalInput(7);


    public static int get() {
        int command = 0;
        boolean moveRobotRight = (!frontRight.get() || !backRight.get());

        boolean moveRobotLeft = (!frontLeft.get() || !backLeft.get());
        boolean onLine = (!frontMid.get() || !backMid.get());
        if (onLine) { //Don't move because sensors found line
            command = 3;
        } else if (moveRobotLeft) { //Move robot left
            command = 1;
        } else if (moveRobotRight) { //Move robot right
            command = 2;
        } else {
            command = 0; //Couldn't find line
        }

        SmartDashboard.putBoolean("0", !frontRight.get());
        SmartDashboard.putBoolean("1", !frontMid.get());
        SmartDashboard.putBoolean("2", !frontLeft.get());
        SmartDashboard.putBoolean("3", !backLeft.get());
        SmartDashboard.putBoolean("4", !backMid.get());
        SmartDashboard.putBoolean("5", !backRight.get());

        SmartDashboard.putBoolean("Move Robot Left", moveRobotLeft);
        SmartDashboard.putBoolean("Move Robot Right", moveRobotRight);
        SmartDashboard.putBoolean("Cargo Aligned", onLine);
        return command;
    }
}
