package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private static CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    private static CANEncoder encoder = new CANEncoder(motor);
    private static CANPIDController pid = new CANPIDController(motor);
    private  static double lastPos = 13.0;
    private  static double pos = 13.0;

    private static double motorOffTime = -10.0;

    public static void init() {
        lastPos = 13.0;
    }

    public static void setPower(double set) {
        motor.set(set*0.4);
//        System.out.println(encoder.getPosition());
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }
//Add safeguards. Set upper and lower limits where the elevator cannot be commanded past
    //Use button to set position to current
    public static void setPosition(double stickY, double tune, double stickX, boolean override, boolean buttonX, int auto) {
        tune *= 10.0; //This changes between hatch setpoints and cargo setpoints
        //When pressed, tune will add 5 to each setpoint

        if (buttonX) {
            pos = 4.0 + tune;
        } else if (stickY >= 0.5) {
            pos = 13.0 + tune;
        } else if (stickX >= 0.5) {
            pos = 56.0 + tune;
        } else if (stickY <= -0.5) {
            pos = 95.5 + tune;
        } else {
            pos = lastPos;
    }
//        System.out.println(auto);
        if (auto == 1) {
            pos = 13.0 + tune;
        } else if (auto == 2) {
            pos = 56.0 + tune;
        } else if (auto ==3) {
            pos  = 95.5 + tune;
        }

        lastPos = pos;

        if (pos < getPosition()) {
            pid.setP(0.03);
        } else {
            pid.setP(0.09);
        }
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);
        pid.setSmartMotionMaxAccel(30.0, 0);
        pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", pos);
        SmartDashboard.putNumber("Motor Temp", motor.getMotorTemperature());
        SmartDashboard.putNumber("Motor Off Time", motorOffTime);
        if (Time.get()-motorOffTime > 10.0) {
            if (override) {
                setPower(-stickY);
            } else {
                pid.setReference(pos, ControlType.kPosition);
            }
        }

        if (motor.getMotorTemperature() > 100.0) {
            System.out.println("Elevator motor off  " + motor.getMotorTemperature() +  "| |" + motor.getAppliedOutput());
            motorOffTime = Time.get();
            motor.set(0.0);
            SmartDashboard.putBoolean("Motor Warning", true);
        } else if (motor.getMotorTemperature() > 65.0) {
//            SmartDashboard.putBoolean("Motor Warning", true);
            System.out.println(motor.getMotorTemperature() +  "| |" + motor.getAppliedOutput());

        } else {
            SmartDashboard.putBoolean("Motor Warning", false);
        }
    }

    public static double getPosition() {
        return encoder.getPosition();
    }
    public static boolean atPosition() {
        if (Math.abs(encoder.getPosition()-pos) < 2.0) {
            return true;
        } else {
            return false;
        }
    }
}