package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hatch {
    private static DoubleSolenoid beak = new DoubleSolenoid(2, 7);
    private static DoubleSolenoid push = new DoubleSolenoid(3,6);
    private static DoubleSolenoid ground = new DoubleSolenoid(5,4);

    private static double prevTime = 0.0;
    private static int step = 0;
    private static boolean hasHatch = true;

    public static void set(boolean grab, boolean place, boolean receive) {

        switch (step) {
            case 0: //Default holding position
                beak.set(Value.kForward);
                push.set(Value.kReverse);
                if (grab) {
                    prevTime = Time.get();
                    step = 1;
                    hasHatch = false;
                } else if (place) {
                    step = 4;
                    hasHatch = true;
                } else if (receive) {
                    step = 8;
                }
                break;
            //Grab hatch
            case 1:  //Close beak
                beak.set(Value.kReverse);
                if (Time.get() - prevTime > 0.1) {
                    prevTime = Time.get();
                    step = 2;
                }

                break;
            case 2: //Push piston forward
                push.set(Value.kForward);
                if (Time.get() - prevTime > 0.15) {
                    prevTime = Time.get();
                    step = 3;
                }
                break;
            case 3: //Open beak and return to default position
                beak.set(Value.kForward);
                if (Time.get() - prevTime > 0.1) {
                    prevTime = Time.get();
                    step = 7;
                }
                break;
            case 7:
                push.set(Value.kReverse);
                if (Time.get() - prevTime > 0.1) {
                    prevTime = Time.get();
                    hasHatch = true;
                    step = 0;
                }
                break;
            //Put hatch
            case 4: //Push piston forward
                push.set(Value.kForward);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 5;
                }
                break;
            case 5: //Close beak
                beak.set(Value.kReverse);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 6;
                }
                break;
            case 6: //Pull piston back and reset to default position
                push.set(Value.kReverse);
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 0;
                    hasHatch = false;
                }
                break;

            case 8:
                beak.set(Value.kReverse);
                if (!receive) {
                    if (Time.get() - prevTime > 1.5) {
                        step = 0;
                        Elevator.setPosition(1.0, 0.0, 0.0,  false, false,  0, false);
                        hasHatch = true;
                    }
                } else {
                    prevTime = Time.get();
                }
                break;
        }
    }

    public static void ground(boolean lift) {
        if (lift) {
            ground.set(Value.kForward);
        } else {
            ground.set(Value.kReverse);
        }

    }

    public static boolean get() {
        return hasHatch;
    }
}