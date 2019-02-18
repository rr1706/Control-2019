package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hatch {
    private static DoubleSolenoid beak = new DoubleSolenoid(0,1);
    private static DoubleSolenoid push = new DoubleSolenoid(2,3);

    private static double prevTime = 0.0;

    public static void set(boolean grab, boolean place) {
        int step = 0;

        switch (step) {
            case 0:
                beak.set(Value.kForward);
                push.set(Value.kReverse);
                if (grab) {
                    prevTime = Time.get();
                    step = 1;
                } else if (place) {
                    step = 4;
                }
                break;
            case 1:
                beak.set(Value.kReverse);
                if (Time.get() - prevTime > 0.1) {
                    prevTime = Time.get();
                    step = 2;
                }
                break;
            case 2:
                push.set(Value.kForward);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 3;
                }
                break;
            case 3:
                beak.set(Value.kForward);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 0;
                }
                break;
            case 4:
                push.set(Value.kForward);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 5;
                }
                break;
            case 5:
                beak.set(Value.kReverse);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 6;
                }
                break;
            case 6:
                push.set(Value.kReverse);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 0;
                }
                break;
        }  
    }
}