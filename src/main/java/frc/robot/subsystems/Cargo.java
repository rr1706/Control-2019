package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Cargo {
    private static TalonSRX motor = new TalonSRX(10);
    private static DoubleSolenoid piston1 = new DoubleSolenoid(4, 5);
    private static DoubleSolenoid piston2 = new DoubleSolenoid(6, 7);

    public static void set(boolean in, boolean out){
        if (in) {
            motor.set(ControlMode.Current, 0.5);
            piston1.set(Value.kForward);
            piston2.set(Value.kForward);
        } else if (out) {
            motor.set(ControlMode.Current, -0.5);
            piston1.set(Value.kReverse);
            piston2.set(Value.kReverse);
        } else {
            motor.set(ControlMode.Current, 0.0);
            piston1.set(Value.kReverse);
            piston2.set(Value.kReverse);
        }
    }
}