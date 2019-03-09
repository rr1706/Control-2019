package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class LineSensor {
    private static DigitalInput side = new DigitalInput(8);
    private static DigitalInput middle = new DigitalInput(9);
    
    public static boolean getSide() {
        return side.get();
    }

    public static boolean getMiddle() {
        return middle.get();
    }
}
