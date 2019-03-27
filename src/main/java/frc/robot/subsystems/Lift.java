package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lift {
    private static TalonSRX motor1 = new TalonSRX(13);
    private static TalonSRX motor2 = new TalonSRX(14);
    private static Encoder liftEncoder;
    private static int climbCase = 0;
    private static boolean manualMode = false;


    public static void init() {
        liftEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
        liftEncoder.setDistancePerPulse(1.0);
        motor1.configOpenloopRamp(0.5);
        motor2.configOpenloopRamp(0.5);
        motor2.follow(motor1);
        manualMode = false;
    }

    public static void climb(double joystick){ //At -65, go to 20% pwr, at 124, stop, 133 is max
        if (!manualMode) {
            switch (climbCase) {
                case 0:
                    motor1.set(ControlMode.PercentOutput, 1.0);
//                motor2.set(ControlMode.PercentOutput, 1.0);
                    if (getDistance() >= 65.0) { //65
                        motor1.configOpenloopRamp(0.0);
                        motor2.configOpenloopRamp(0.0);
                        climbCase = 1;
                    }
                    break;
                case 1:
                    motor1.set(ControlMode.PercentOutput, 0.2);
//                    motor2.set(ControlMode.PercentOutput, 0.2);
                    if (getDistance() >= 115.0) { //124
                        climbCase = 2;
                    }
                    break;
                case 2:
                    motor1.set(ControlMode.PercentOutput, 0.0);
                    break;
            }
        } else {
            if (Math.abs(joystick) < 0.1) {
                joystick = 0.0;
            }
            motor1.set(ControlMode.PercentOutput, -joystick);
        }
    }

    public static void setManual() {
        manualMode = true;
    }

    public static double getDistance() {
        return -liftEncoder.getDistance();
    }
}