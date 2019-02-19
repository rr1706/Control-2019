package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.*;
//import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class BallIntake{
    private static DoubleSolenoid ball = new DoubleSolenoid(4, 5);
    private static int state = 0;
    private static TalonSRX BallMotor = new TalonSRX(10);

    public static void set(double speed){
        if(Robot.xboxEl.X() == true){
            state = 1;
        }
        if(Robot.xboxEl.Y() == true){
            state = 2;
        }
        switch (state){
            case 0 :
                break;
            case 1 :
                ball.set(DoubleSolenoid.Value.kForward);
                break;
            case 2 :
                ball.set(DoubleSolenoid.Value.kReverse);
                break;
        }
        if(Robot.xboxEl.RTrig() >= 0.5){
            BallMotor.set(ControlMode.PercentOutput, speed);
        }
        if(Robot.xboxEl.LTrig() >= 0.5){
            BallMotor.set(ControlMode.PercentOutput, -speed);
        }
    }
}