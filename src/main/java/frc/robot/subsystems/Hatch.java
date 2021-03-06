package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Hatch {
    private static DoubleSolenoid beak = new DoubleSolenoid(7, 2);
    private static DoubleSolenoid push = new DoubleSolenoid(3,1);
//    private static DoubleSolenoid ground = new DoubleSolenoid(50,40);

    private static double prevTime = 0.0;
    private static boolean startingHatch = false;
    private static int step = 0;
    private static boolean hasHatch = true;
    private static  boolean running = false;

    public static boolean set(boolean grab, boolean place, boolean receive, boolean safe) {
        switch (step) {
            case 0: //Default holding position
                beak.set(Value.kForward);
                push.set(Value.kReverse);
                running = false;
                if (grab) {
                    running = true;
                    prevTime = Time.get();
                    step = 1;
                    hasHatch = false;
                } else if (place/* && safe*/) {
//                    putting = true;
                    running = true;
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
//                System.out.println("Going Back");
                if (Time.get() - prevTime > 0.2) {
                    prevTime = Time.get();
                    step = 9;
                    hasHatch = false;
//                    running = false; //FIXME Do the same for the intake
                }
                break;

            case 9:
                running = false;
                push.set(Value.kReverse);
                beak.set(Value.kReverse);

                if (Time.get() - prevTime > 2.0) {
                    step = 0;
                }

                if (grab) {
                    running = true;
                    prevTime = Time.get();
                    step = 1;
                    hasHatch = false;
                } else if (place && safe) {
                    running = true;
                    step = 4;
                    hasHatch = true;
                } else if (receive) {
                    step = 8;
                }

                break;
            case 8:
                beak.set(Value.kReverse);
                if (!receive) {
                    if (Time.get() - prevTime > 1.5) {
                        step = 0;
                        Elevator.setPosition(1.0, 0.0, 0.0,  false, false, false);
                        hasHatch = true;
                    }
                } else {
                    prevTime = Time.get();
                }
                break;
        }
        return running;
    }

    public static void ground(boolean lift) {
        if (lift) {
//            ground.set(Value.kForward);
        } else {
//            ground.set(Value.kReverse);
        }

    }

    public static boolean hasHatch() {
        return hasHatch;
    }

    public static boolean isRunning() {
        return running;
    }
}