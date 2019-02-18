package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {
    private static CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    private static CANEncoder encoder = new CANEncoder(motor);
    private static CANPIDController pid = new CANPIDController(motor);

    public static void setPower(double set) {
        motor.set(set*0.5);
        System.out.println(encoder.getPosition());
    }

    public static void setPosition(double pos) {
        pid.setP(0.001);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);
        pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        pid.setReference(pos, ControlType.kPosition);
    }
}