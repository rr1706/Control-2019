package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.Vector;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * A swerve module.
 */
public class SwerveModule {
    private static final double TICKS_PER_REVOLUTION = 360.0;
    private static final double DISTANCE_PER_PULSE = 1.0;//0.91;

    private Vector position;
    private boolean disableSpeed = false;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double previousDistance = 0;
    private double previousRobotDistance = 0.0;
    private double trueError;
    private double rawError;
    private double delta = 0.0;
    private double rightDelta = 0.0;
    private double forwardDelta = 0.0;
    private double rightSum = 0;
    private double forwardSum = 0;
    private SwerveMotor swerveMotor;
    private PIDController anglePID;
    private double keepPIDAngle;
    private double angle = 0.0;
    private boolean wheelReversed;
    private double reversePoint;
    private DigitalInput wheelSensor;
    private double wheelOffsetFirst = 0.0;
    private double wheelOffset = 0.0;
    private double sensorCounter = 0.0;
    private  double testing = 0.0;

    public static XboxController xbox1 = new XboxController(0);

    private double angleOld;
    private double angleFiltered = 0.0;
    private double angleFilteredOld = 0.0;

    private boolean resettingAngle = false;
    private boolean foundSensor = false;
    private boolean foundFlag = false;

//Good base values:
//    private double kP = 1.45e-4;
//    private double kI = 1.6e-6;
//    private double kD = 8.0e-5;

    private double kP = 1.5e-3;//8.0e-4; //1.8, 1.3e-3
    private double kI = 0.0;
    private double kD = 5.7e-5/*6.4e-5*/; //5.9e-5

    private double maxRPM = 5676;

//    private double omega_dRPM = 138.4615/400.0;
    private double Max_dRPM = 400.0;
//    private double max_omega = omega_dRPM * Max_dRPM;
//    private double alpha_er = 0.0;
//    private double raw_turn_time = 0.0;
//    private double dT = 0.02;
//    private double turn_command_counts = 0.0;
//    private double omega_command = 0.0;
//    private double dRPM_command = 0.0;


    private double rotationCommand = 0.0;


    double spin = 0;

    private int id;


    /**
     * @param canPortC Port of the motor that moves the wheel Clockwise
     * @param canPortCC Port of the motor that moves the wheel CounterClockwise
     * @param sensorPort Roborio port of the allignment sensor
     */
    SwerveModule(int canPortC, int canPortCC, int sensorPort) {
        super();

        swerveMotor = new SwerveMotor(canPortC, canPortCC);

        anglePID = new PIDController(0.0, 0.0, 0.0);
        anglePID.setContinuous();
        anglePID.setInputRange(0.0, 360.0);
        anglePID.setOutputRange(-1.0, 1.0);
        anglePID.enable();

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
        wheelSensor = new DigitalInput(sensorPort);
    }

    void drive() {
        distance = swerveMotor.getDistance() * DISTANCE_PER_PULSE;

//        wheelOffset = 0.0;

        angleOld = angle;
        //Low Pass Filter
        angle = MathUtils.resolveDeg(/*wheelOffset*/swerveMotor.getAngle()); //FIXME!!! This will cause error unless it is initiated by the button that rotates them all in the same direction!
        //Wheel offset numbers are different between the comp and practice robots
        angleFilteredOld = angleFiltered;

        if (angle-angleOld >= 45) {
            angleOld += 360;
        } else if (angleOld-angle >= 45) {
            angleOld -= 360;
        }

        angleFiltered = MathUtils.resolveDeg((angle+angleOld+9*angleFilteredOld)/11);

        trueError = MathUtils.calculateContinuousError(angleCommand, angle, 360.0, 0.0);

//        SmartDashboard.putNumber("Combined Rates of Change", angleCommand*angle);

        if (id == 1) {
            SmartDashboard.putNumber("FR Angle Command", angleCommand);
//            SmartDashboard.putNumber("FR Speed Command", speedCommand);
//            SmartDashboard.putNumber("FR Angle", swerveMotor.getAngle());
//            SmartDashboard.putNumber("FR Angle Error", trueError);
//            SmartDashboard.putNumber("FR Raw Angle Error", rawError);
            SmartDashboard.putBoolean("FR Sensor", !wheelSensor.get());

            SmartDashboard.putNumber("FR Offset", wheelOffset);
            SmartDashboard.putNumber("FR Calculated Angle", MathUtils.resolveDeg(wheelOffset+swerveMotor.getAngle()));
//            SmartDashboard.putNumber("FR Sensor Counts", sensorCounter);

            SmartDashboard.putNumber("FR Distance", distance);

            if (!wheelSensor.get()) {
                SmartDashboard.putNumber("FR Sensor Angle ",  angle);
//
//                if (wheelOffsetFirst == 0.0) {
//                    wheelOffsetFirst = angle;
//                } else {
//                    wheelOffset = wheelOffsetFirst-angle;
//                }
            }
        } else if (id == 2) {
            SmartDashboard.putNumber("FL Angle Command", angleCommand);
//            SmartDashboard.putNumber("FL Filtered Angle", angleFiltered);
            SmartDashboard.putNumber("FL Speed Command", speedCommand);
//            SmartDashboard.putNumber("FL Angle", swerveMotor.getAngle());
////            SmartDashboard.putNumber("FL Raw Angle", swerveMotor.getRawAngle());
//            SmartDashboard.putNumber("FL Angle Error", trueError);
//            SmartDashboard.putNumber("FL Raw Angle Error", rawError);
            SmartDashboard.putBoolean("FL Sensor", !wheelSensor.get());

            SmartDashboard.putNumber("FL Offset", wheelOffset);
            SmartDashboard.putNumber("FL Calculated Angle", MathUtils.resolveDeg(wheelOffset+swerveMotor.getAngle()));
//            SmartDashboard.putNumber("FL Sensor Counts", sensorCounter);

            SmartDashboard.putNumber("FL Distance", distance);


            if (!wheelSensor.get()) {
                SmartDashboard.putNumber("FL Sensor Angle ",  angle);
//                if (wheelOffsetFirst == 0.0) {
//                    wheelOffsetFirst = angle;
//                } else {
//                    wheelOffset = wheelOffsetFirst-angle;
//                }
            }

        } else if (id == 3) {
//            SmartDashboard.putNumber("BR Angle", angle);
            SmartDashboard.putNumber("BR Angle Command", angleCommand);
            SmartDashboard.putNumber("BR Speed Command", speedCommand);
//            SmartDashboard.putNumber("BR Angle Error", trueError);
//            SmartDashboard.putNumber("BR Raw Angle Error", rawError);
//            SmartDashboard.putBoolean("BR Sensor", !wheelSensor.get());

            SmartDashboard.putBoolean("BR Sensor", !wheelSensor.get());
            SmartDashboard.putNumber("BR Offset", wheelOffset);
            SmartDashboard.putNumber("BR Calculated Angle", MathUtils.resolveDeg(wheelOffset+swerveMotor.getAngle()));
//            SmartDashboard.putNumber("BR Sensor Counts", sensorCounter);

            SmartDashboard.putNumber("BR Distance", distance);

            if (!wheelSensor.get()) {
                SmartDashboard.putNumber("BL Sensor Angle ",  angle);
//
//                if (wheelOffsetFirst == 0.0) {
//                    wheelOffsetFirst = angle;
//                } else {
//                    wheelOffset = wheelOffsetFirst-angle;
//                }
            }

        } else {
//            SmartDashboard.putNumber("BL Angle", angle);
            SmartDashboard.putNumber("BL Angle Command", angleCommand);
            SmartDashboard.putNumber("BL Speed Command", speedCommand);
//            SmartDashboard.putNumber("BL Angle Error", trueError);
//            SmartDashboard.putNumber("BL Raw Angle Error", rawError);
            SmartDashboard.putBoolean("BL Sensor", !wheelSensor.get());

//            SmartDashboard.putBoolean("BL Sensor", !wheelSensor.get());

            SmartDashboard.putNumber("BL Offset", wheelOffset);
            SmartDashboard.putNumber("BL Calculated Angle", MathUtils.resolveDeg(wheelOffset+swerveMotor.getAngle()));
//            SmartDashboard.putNumber("BL Sensor Counts", sensorCounter);

            SmartDashboard.putNumber("BL Distance", distance);

            if (!wheelSensor.get()) {
                SmartDashboard.putNumber("BR Sensor Angle ",  angle);

//                if (wheelOffsetFirst == 0.0) {
//                    wheelOffsetFirst = angle;
//                } else {
//                    wheelOffset = wheelOffsetFirst-angle;
//                }
            }
        }

        // if (wheelReversed) {
            delta = previousDistance - distance;
        // } else {
        //     delta = distance - previousDistance;
        // }

        /*
         * If the wheel has to move over 90 degrees
         * go opposite to command and reverse translation
         */

        //FIXME, the angleCommand will flip 180 even without this logic at times

//            if (Math.abs(trueError) > TICKS_PER_REVOLUTION / 4.0) {
//                angleCommand = MathUtils.reverseWheelDirection(angleCommand);
//                speedCommand *= -1;
//                wheelReversed = true;
//            } else {
//                wheelReversed = false;
//            }

        anglePID.setPID(SmartDashboard.getNumber("kP", kP/*0.9e-3, 1e-3*/), SmartDashboard.getNumber("kI", kI/*6.8e-6, 1e-5*/), SmartDashboard.getNumber("kD", kD/*1.9e-4, 2e-4*/));

//        kP = SmartDashboard.getNumber("kP", kP);
//        kI = SmartDashboard.getNumber("kI", kI);
//        kD = SmartDashboard.getNumber("kD", kD);

/* TODO Old offsets:
front_right_drift=-0.0051,-0.009
front_left_drift=0.00236,-0.004
back_left_drift= -0.0025,-0.0085
back_right_drift=0.0059,0.0027
 */
        anglePID.setInput(angle);


//        anglePID.setSetpoint(90*speedCommand);


//        if (id == 4) {
//            System.out.println(trueError + "||" + angle + "||" + angleCommand);
//        }


//        alpha_er = trueError;

        if (Math.abs(speedCommand) > 0.01) { //KeepAngle for modules
            anglePID.setSetpoint(angleCommand);
        } else {
            speedCommand = 0.1;
        }

        /*
         * If wheel direction has to change more than 22.5 degrees
         * then set wheel speed command to 0 while wheel is turning.
         */

//        if (Math.abs(anglePID.getError()) > TICKS_PER_REVOLUTION / 16) {
//            speedCommand = 0.0;
//        }

//        raw_turn_time = Math.abs(alpha_er/max_omega);
//        turn_command_counts = Math.ceil(raw_turn_time/dT);
//        omega_command = alpha_er/(turn_command_counts*dT);
//        dRPM_command = omega_command/omega_dRPM;
//        rotationCommand = dRPM_command/maxRPM;
//

//        rotationCommand = Math.signum(trueError)*Math.pow(Math.signum(trueError)*trueError, 0.99)/540.0;
//        if (Math.abs(rotationCommand) >= 0.7) {
//            rotationCommand = Math.signum(rotationCommand)*0.7;
//        }
//
//        if (Math.abs(trueError) <= 2.0) {
//            rotationCommand = 0.0;
//        }


        if (resettingAngle && wheelSensor.get()) {
            rotationCommand = 0.03;
            speedCommand = 0.0;
//            System.out.println("Looking For Wheels");
        } else if  (resettingAngle && !wheelSensor.get()) {
            resettingAngle = false;
            rotationCommand = 0.0;

            if (id == 1) {
                wheelOffset = MathUtils.resolveDeg(259.1312255859375 - angle); //Change
//                System.out.println("Wheel 1 Found");
                SmartDashboard.putBoolean("Wheel 1 Aligned", true);

            }else if (id == 2) {
                wheelOffset = MathUtils.resolveDeg(73.0799560546875 - angle); //Change
//                System.out.println("Wheel 2 Found");
                SmartDashboard.putBoolean("Wheel 2 Aligned", true);


            }else if (id == 3) {
                wheelOffset = MathUtils.resolveDeg(253.8753662109375 - angle);
//                System.out.println("Wheel 3 Found");
                SmartDashboard.putBoolean("Wheel 3 Aligned", true);


            } else if (id == 4) {
                wheelOffset = MathUtils.resolveDeg(77.2393798828125 - angle);
//                System.out.println("Wheel 4 Found");
                SmartDashboard.putBoolean("Wheel 4 Aligned", true);
            }
        } else {
            rotationCommand = anglePID.performPID();
        }

//        System.out.println(rotationCommand);
//        if (id == 3) {
//        rotationCommand = joerot/maxRPM;
//        joerot = joerot + 0.02;

//            rotationCommand = xbox1.RStickX()/20;
//            if (Math.abs(rotationCommand) < 0.008) {
//                rotationCommand = 0.0;
//            }


//        if (id == 1) {
//            System.out.println(angleCommand);

//        System.out.println(testing);
//            System.out.println(speedCommand + "||" + testing);

        if (disableSpeed) { //FIXME, use this to cut speedCommand when a wheel is not at the right angle
            speedCommand = 0.0;
        }
        swerveMotor.set(speedCommand,  rotationCommand);
//            System.out.println(speedCommand);
//        } else {
//            swerveMotor.set(0.0, 0.0);
//        }


/*
front_right_drift=-0.001,-0.001
front_left_drift=0.001,0.0
back_left_drift=-0.002,-0.002
back_right_drift=0.0028,0.0028
 */

        rightDelta = delta * Math.sin(Math.toRadians(angle));
        forwardDelta = delta * Math.cos(Math.toRadians(angle));




//        if (id == 3) {
//            SmartDashboard.putNumber("BL Angle Command", anglePID.performPID());
//            SmartDashboard.putNumber("BL Angle", angle);
//            SmartDashboard.putNumber("BL Error", trueError);
//
//        }

        previousDistance = distance;
        previousRobotDistance = SwerveDrivetrain.getRobotDistance();
    }

    public void setID(int id) {
        this.id = id;
        swerveMotor.setID(id);
    }

//    public void setDrift(String[] drift) {
//        swerveMotor.setDrift(drift);
//    }

    public void setOffset(double set) {
        wheelOffsetFirst = set;
    }
    public double getAngle() {
        return angle;
    }

    public void setSpeedCommand(double speedCommand) {
        this.speedCommand = speedCommand;
    }

    public void disableSpeed(boolean disableSpeed) {
        this.disableSpeed = disableSpeed;
    }

    public void setRotationCommand(double rotationCommand) {
        this.testing = rotationCommand;
    }
    void setAngleCommand(double angleCommand) {
        this.angleCommand = angleCommand;
    }

    void resetAngle (boolean set) {
        resettingAngle = set;

        if (!foundFlag) {
            foundSensor = false;
            foundFlag = true;
            SmartDashboard.putBoolean("Wheel 1 Aligned", false);
            SmartDashboard.putBoolean("Wheel 2 Aligned", false);
            SmartDashboard.putBoolean("Wheel 3 Aligned", false);
            SmartDashboard.putBoolean("Wheel 4 Aligned", false);
        }
    }

    public void setFoundFlag (/*boolean startButton*/) {
        foundFlag= false;
//        buttonDown = startButton;
    }

    double getSpeedCommand() {
        return speedCommand;
    }

    public double getAngleCommand() {
        return angleCommand;
    }

    Vector getPosition() {
        return position;
    }

    public void setPosition(Vector position) {
        this.position = position;
    }

    public boolean getAngleOff() {
//        if (wheelReversed) {
//            angle = MathUtils.resolveDeg(angle + 180);
//        }
        double angleError = Math.abs(MathUtils.getAngleError(angle, angleCommand));
        if  (Math.abs(MathUtils.getAngleError(angle, Math.abs(MathUtils.reverseWheelDirection(angleCommand)))) < Math.abs(MathUtils.getAngleError(angle, angleCommand))) {
            angleError = Math.abs(MathUtils.getAngleError(angle, Math.abs(MathUtils.reverseWheelDirection(angleCommand))));
        }

        return (angleError > 25.0);
    }

    public double getDistance() {
        return distance;
    }

    public void resetDistance() {
        swerveMotor.reset();
    }

    public boolean getReversed() {
        return this.wheelReversed;
    }

    public double getRightSum() {
        return rightSum;
    }

    public double getForwardSum() {
        return forwardSum;
    }

	public void resetDelta() {
		rightSum = 0.0;
		forwardSum = 0.0;
	}

	public double[] getXYDist() {
		if (wheelReversed) {
			forwardSum *= -1.0;
			rightSum *= -1.0;
		}
		double[] i = {rightDelta, forwardDelta};
		return i;
	}
}
