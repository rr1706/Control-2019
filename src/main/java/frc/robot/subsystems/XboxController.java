package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick {
	private Joystick stick;
	private double deadband;

	public XboxController(int port) {
		super(port);
		stick = new Joystick(port);
	}

	public boolean A() {
		return stick.getRawButton(1);
	}

	public boolean aPressed() {
		return stick.getRawButtonPressed(1);
	}

	public boolean B() {
		return stick.getRawButton(2);
	}

	public boolean X() {
		return stick.getRawButton(3);
	}

	public boolean Y() {
		return stick.getRawButton(4);
	}

	public boolean LB() {
		return stick.getRawButton(5);
	}

	public boolean RB() {
		return stick.getRawButton(6);
	}

	public boolean Back() {
		return stick.getRawButton(7);
	}

	public boolean Start() {
		return stick.getRawButton(8);
	}

	public boolean LStickButton() {
		return stick.getRawButton(9);
	}

	public boolean RStickButton() {
		return stick.getRawButton(10);
	}

	public double LStickX() {
		if (Math.abs(stick.getRawAxis(0)) > deadband) {
			return stick.getRawAxis(0);
		} else {
			return 0.0;
		}
	}

	public double LStickY() {
		if (Math.abs(stick.getRawAxis(1)) > deadband) {
			return stick.getRawAxis(1);
		} else {
			return 0.0;
		}
	}

	public double LTrig() {
		return stick.getRawAxis(2);
	}

	public double RTrig() {
		return stick.getRawAxis(3);
	}

	public double RStickX() {
		if (Math.abs(stick.getRawAxis(4)) > deadband) {
			return stick.getRawAxis(4);
		} else {
			return 0.0;
		}
	}

	public double RStickY() {
		if (Math.abs(stick.getRawAxis(5)) > deadband) {
			return stick.getRawAxis(5);
		} else {
			return 0.0;
		}
	}

	public int DPad() {
		return stick.getPOV();
	}

	public int buttonPad() {
		if (Y() && B()) {
			return 45;
		} else if (B() && A()) {
			return 135;
		} else if (A() && X()) {
			return 225;
		} else if (X() && Y()) {
			return 315;
		} else if (Y()) {
			return 0;
		} else if (B()) {
			return 90;
		} else if (A()) {
			return 180;
		} else if (X()) {
			return 270;
		} else {
			return -1;
		}
	}

	public void setDeadband(double band) {
		this.deadband = band;
	}

	public void rumbleRight(double val) {
		stick.setRumble(RumbleType.kRightRumble, val);
	}

	public void rumbleLeft(double val) {
		stick.setRumble(RumbleType.kLeftRumble, val);
	}

	public void stopLeftRumble() {
		stick.setRumble(RumbleType.kLeftRumble, 0);
	}

	public void stopRightRumble() {
		stick.setRumble(RumbleType.kRightRumble, 0);
	}
}
