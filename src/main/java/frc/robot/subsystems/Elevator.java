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
    private static int motorSafety = 0;

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
    public static boolean setPosition(double stickY, double tune, double stickX, boolean override, boolean buttonX, boolean hatch) {
        tune *= 10.0; //This changes between hatch setpoints and cargo setpoints
        //When pressed, tune will add 5 to each setpoint

        if (buttonX) {
            pos = 4.0 + tune;
        } else if (stickY >= 0.5) {
            pos = 9.0 + tune;
        } else if (Math.abs(stickX) >= 0.5) {
            pos = 54.0 + tune;
        } else if (stickY <= -0.5) {
            pos = 95.5 + tune;
            if (pos > 100.0) {
                pos = 100.0;
            }
        } else if (lastPos == 4.0){
            pos = 13.0;
            lastPos = 13.0;
        } else {
            pos = lastPos;
    }

        if (override) {
            pos = lastPos;
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
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", pos);
        SmartDashboard.putNumber("Motor Temp", motor.getMotorTemperature());
        SmartDashboard.putNumber("Motor Off Time", motorOffTime);

//        System.out.println(pos + " || " + encoder.getPosition());

        switch (motorSafety) {
            case 0:
                    pid.setReference(pos, ControlType.kPosition);
                if (motor.getMotorTemperature() >= 90.0) {
                    motorSafety = 1;
                }
                break;

            case 1:
                System.out.println("Elevator Disabled: Temperature");
                motor.set(0.0);
                if (motor.getMotorTemperature() < 80.0) {
                    motorSafety = 0;
                }
                break;
        }

       if (motor.getMotorTemperature() > 65.0) {
            SmartDashboard.putBoolean("Motor Warning", true);
        } else {
            SmartDashboard.putBoolean("Motor Warning", false);
        }

       return atPosition();
    }

    public static double getPosition() {
        return encoder.getPosition();
    }

    public static double getCommand() {
        return pos;
    }


    public static boolean atPosition() {
        return (Math.abs(encoder.getPosition()-pos) < 2.0);
    }

//    public static boolean liftingCargo() {
//        System.out.println(pos + "  " + lastPos);
//        return (!atPosition() && lastPos == 4.0);
//    }
}