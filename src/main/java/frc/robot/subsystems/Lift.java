package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lift {
    private static TalonSRX motor1 = new TalonSRX(13);
    private static TalonSRX motor2 = new TalonSRX(14);
    private static Encoder liftEncoder;

    public static void climb(boolean start){
        liftEncoder = new Encoder(15, 16, false, Encoder.EncodingType.k4X);
        liftEncoder.setDistancePerPulse(1.0);
        int climbCase = 0;
        motor1.configOpenloopRamp(0.5);
        motor2.configOpenloopRamp(0.5);
        switch (climbCase){
            case 0:
                motor1.set(ControlMode.PercentOutput, 1.0);
                motor2.set(ControlMode.PercentOutput, 1.0);
                if(liftEncoder.getDistance() >= 5.0){
                    motor1.configOpenloopRamp(0.0);
                    motor2.configOpenloopRamp(0.0);
                    climbCase = 1;
                }
                break;
            case 1:
                motor1.set(ControlMode.PercentOutput, 0.2);
                motor2.set(ControlMode.PercentOutput, 0.2);
                if(liftEncoder.getDistance() >= 6.0){
                    climbCase = 2;
                }
                break;
            case 2:
                motor1.set(ControlMode.PercentOutput, 0.0);
                motor2.set(ControlMode.PercentOutput, 0.0);
                SmartDashboard.putBoolean("Climb Done", true);
                break;
        }
    }
}