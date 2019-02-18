package frc.robot.subsystems;

//import java.lang.*;
//import java.util.*;
import frc.robot.*;
//import java.sql.Time;

import edu.wpi.first.wpilibj.*;

public class HatchIntake{
    
    private static DoubleSolenoid hatch = new DoubleSolenoid(0, 1);
    private static DoubleSolenoid arm = new DoubleSolenoid(2, 3);
 //private static DoubleSolenoid ball = new DoubleSolenoid(4, 6);
    private static double prevTime = 0;

    public static void set(int state){
        if(Robot.xboxEl.B() == true){
            state = 1;
        }
        if(Robot.xboxEl.A() == true){
            state = 2;
        }
        switch (state) {
            case 0 :
                break;
            case 1 :
                arm.set(DoubleSolenoid.Value.kForward);
                if((frc.robot.subsystems.Time.get() - prevTime) == .25)
                {
                    hatch.set(DoubleSolenoid.Value.kForward);
                    if((frc.robot.subsystems.Time.get() - prevTime) == .5){
                        arm.set(DoubleSolenoid.Value.kReverse);
                    }
                }
                state = 0;
                break;    
            case 2 :
                arm.set(DoubleSolenoid.Value.kForward);
                if((frc.robot.subsystems.Time.get() - prevTime) == .25)
                {
                    hatch.set(DoubleSolenoid.Value.kReverse);
                    if((frc.robot.subsystems.Time.get() - prevTime) == .5){
                        arm.set(DoubleSolenoid.Value.kReverse);
                    }
                }
                state = 0;
                break;    
            }
        prevTime = frc.robot.subsystems.Time.get();
        System.out.println(arm.get());
        System.out.println(hatch.get());
    }







    
}