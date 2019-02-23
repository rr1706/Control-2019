package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private static CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    private static CANEncoder encoder = new CANEncoder(motor);
    private static CANPIDController pid = new CANPIDController(motor);
    private  static double lastPosition = 0.0;

    public static void setPower(double set) { //Ian said to reverse
        motor.set(-set*0.4);
//        System.out.println(encoder.getPosition());
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }

    //Use button to set position to current
    public static void setPosition(double pos) {
        if (lastPosition > pos) {
            pid.setP(0.04);
        } else if (pos == 60)  {
            pid.setP(0.07);
        } else {
            pid.setP(0.10);
        }
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);
        pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());

        pid.setReference(pos, ControlType.kPosition);
        lastPosition = pos;
    }

    public static double getPosition() {
        return encoder.getPosition();
    }
}