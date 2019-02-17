package frc.robot;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class Ds {

	//private static DriverStation ds = DriverStation.getInstance();

	public static double getBatteryVoltage() {
		return RobotController.getBatteryVoltage();
	}
}
