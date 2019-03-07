package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private static CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    private static CANEncoder encoder = new CANEncoder(motor);
    private static CANPIDController pid = new CANPIDController(motor);
    private  static double lastPos = 12.0;

    public static void setPower(double set) { //Ian said to reverse
        motor.set(set*0.4);
//        System.out.println(encoder.getPosition());
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }
//Add safeguards. Set upper and lower limits where the elevator cannot be commanded past
    //Use button to set position to current
    public static void setPosition(double stickY, double tune, double stickX, boolean override) {
        double pos = 0.0;
        tune *= 5.0;

        if (stickY >= 0.5) {
            pos = 12.0 + tune;
        } else if (stickX >= 0.5) {
            pos = 34.0 + tune;
        } else if (stickY <= -0.5) {
            pos =58.2 + tune;
        } else {
            pos = lastPos;
        }
        lastPos = pos;
 
        if (pos < getPosition()) {
            pid.setP(0.045);
        } else {
            pid.setP(0.06);
        }
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);
        pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", pos);

        if  (override) {
            setPower(-stickY);
        } else {
            pid.setReference(pos, ControlType.kPosition);
        }
    }

    public static double getPosition() {
        return encoder.getPosition();
    }
}