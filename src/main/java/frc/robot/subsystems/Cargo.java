package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Cargo {
    private static TalonSRX motor = new TalonSRX(12);
    private static DoubleSolenoid piston1 = new DoubleSolenoid(0, 1);
    private static int step = 0;
    private static int counter = 0;
//    private static Lidar ballSensor = new Lidar();

//    private static DoubleSolenoid piston2 = new DoubleSolenoid(4, 5);

    public static void set(boolean autoIntake, boolean in, boolean out, double distanceToCargo, double leftDistance, double rightDistance){

//        System.out.println(step + "| |" + distanceToCargo);
        if (autoIntake) {
            switch (step) { //Look for cargo
                case 0:
                    motor.set(ControlMode.PercentOutput, 0.0);
                    piston1.set(Value.kReverse);
                    if (leftDistance > 20.0 && rightDistance > 20.0 && distanceToCargo < 30.0) {
                        step = 1;
                    }
                    break;
                case 1:
                        motor.set(ControlMode.PercentOutput, 0.75);
                        piston1.set(Value.kForward);
                        if (distanceToCargo < 10.0) { //If cargo is in the intake,
                            counter++;
                        }
                        if (counter >= 5) {
                            step = 2;
                            counter = 0;
                        }
                        if (distanceToCargo > 30.0) {
                            step = 0;
                        }
                    break;
                case 2: //Pull the intake up and wait for cargo to be fully intaken
                    piston1.set(Value.kReverse);
                    motor.set(ControlMode.PercentOutput, 0.5);
                    if (distanceToCargo > 30.0) {
                        step = 0;
                    }
                    if (distanceToCargo < 5.0) { //If cargo is in the intake,
//                        System.out.println("Here");
                        motor.set(ControlMode.PercentOutput, 0.0);
                    }
                    break;
            }
        }

        if (in) {
            motor.set(ControlMode.PercentOutput, 0.75);
            piston1.set(Value.kForward);
        } else if (out) {
            piston1.set(Value.kReverse);
            motor.set(ControlMode.PercentOutput, -0.75);
            step = 0;
        } else if (!autoIntake){
            motor.set(ControlMode.PercentOutput, 0.0);
            piston1.set(Value.kReverse);
        }
    }
}