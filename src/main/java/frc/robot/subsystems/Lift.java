package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Lift {
    private static TalonSRX motor1 = new TalonSRX(13);
    private static TalonSRX motor2 = new TalonSRX(14);


    public static void climb(boolean in, boolean out){

        if (in) {
            motor1.set(ControlMode.PercentOutput, -0.5);
            motor2.set(ControlMode.PercentOutput, -0.5);

        } else if (out) {
            motor1.set(ControlMode.PercentOutput, 0.5);
            motor2.set(ControlMode.PercentOutput, 0.5);
        }
        else {
            motor1.set(ControlMode.PercentOutput, 0.0);
            motor2.set(ControlMode.PercentOutput, 0.0);
        }
    }
}