package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lift {
    private static TalonSRX motor1 = new TalonSRX(13);
    private static TalonSRX motor2 = new TalonSRX(14);
//    private static DoubleSolenoid release = new DoubleSolenoid(5, 4);
    private static Encoder liftEncoder = new Encoder(4,
        5, false, Encoder.EncodingType.k4X);
    private static int climbCase = 0;
    private static boolean manualMode = false;
    private static double prevTime = 0.0;


    public static void init() {
        liftEncoder.setDistancePerPulse(1.0);
        motor1.configOpenloopRamp(0.5);
        motor2.configOpenloopRamp(0.5);
        motor2.follow(motor1);
//        manualMode = false;
//        release.set(Value.kReverse);
    }

//    public static void reset() { //Currently the code CANNOT RUN lift code multiple times without redeploying
//        release.set(Value.kReverse);
//    }

    public static void climb(double joystick){ //At -65, go to 20% pwr, at 124, stop, 133 is max
//        release.set(Value.kForward);
   manualMode = true;//FIXME, comment out before competition!!!
        if (!manualMode) {
            switch (climbCase) {
                case 0:
//                    motor1.set(ControlMode.PercentOutput, -0.05); //Motors might be reversed, so test slow and in manual mode
                    if (Time.get() - prevTime > 0.1) {
                        climbCase = 1;
                    }
                    break;
                case 1:
                    motor1.set(ControlMode.PercentOutput, 1.0);
                    if (getDistance() >= 72.0) { //65
                        motor1.configOpenloopRamp(0.0);
                        motor2.configOpenloopRamp(0.0);
                        climbCase = 4;
                    }
                    break;
                case 2:
                    motor1.set(ControlMode.PercentOutput, 0.2);
                    if (getDistance() >= 115.0) {
                        climbCase = 3;
                    }
                    break;
                case 3:
                    motor1.set(ControlMode.PercentOutput, 0.0);
                    break;
                case 4:
                    manualMode = true;
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
        return Math.abs(liftEncoder.getDistance());
    }
}