package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Cargo {
    private static TalonSRX motor = new TalonSRX(12);
    private static DoubleSolenoid piston1 = new DoubleSolenoid(6, 0);
    private static int step = 0;
    private static int counter = 0;
    private static int counter2 = 0;
    private static int holdState = 0;

    public static void set(boolean autoIntake, boolean in, boolean out, double distanceToCargo, double leftDistance, double rightDistance){

        if (in) {
            motor.set(ControlMode.PercentOutput, 0.6);
            piston1.set(Value.kForward);
            holdState = 1;
        } else if (out) {
            piston1.set(Value.kReverse);
            motor.set(ControlMode.PercentOutput, -0.75);
            holdState = 2;
//            step = 0;
        } else if (autoIntake){ //Auto stop the intake if the ball is already in
            motor.set(ControlMode.PercentOutput, 0.2);
            piston1.set(Value.kReverse);
            holdState = 3;
        } else {
            piston1.set(Value.kReverse);
            motor.set(ControlMode.PercentOutput, 0.0);
            holdState = 0;
        }
    }

    public static int getHoldState () {
        return holdState;
    }
}